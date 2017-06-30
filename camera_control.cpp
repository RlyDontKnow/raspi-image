// Copyright (c) 2017, Matthieu Kraus
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the
//   names of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "camera_control.h"
#include "scope_guard.h"

#include <cmath>
#include <iterator>
#include <stdexcept>
#include <type_traits>

#include "fcntl.h"
#include "sys/ioctl.h"

#include "linux/i2c.h"
#include "linux/i2c-dev.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

namespace ch = std::chrono;

namespace raspi
{

namespace
{
  namespace constants
  {
    static uint16_t const imager_width = 3280;
    static uint16_t const imager_height = 2464;

    static uint16_t const model_id = 0x0219;
    static uint16_t const model_id_address = 0x0000;

    static double const clock_frequency = 24.0; // [MHz]
    static uint8_t const mipi_lanes = 0x01; // 1 = 2 lanes, 3 = 4 lanes

    // @@@TODO: the pix_clk is too low according to the specification - does that cause any issues?
    //          raising the frequencies to 200 MB/s throughput seems to cause issues
    // sys_clk = 344 MHz (valid range: 200-700)
    // pix_clk = 68 MHz (valid range: 80-140)
    // output rate: 136 MPx/s = 170 MB/s
    // pll1_clk = clock_frequency * video_timing_clock_mul / video_timing_pre_pll_clock_div
    // sys_clk = pll1_clk / video_timing_system_clock_div
    // pix_clk = pll1_clk / video_timing_system_clock_div
    static uint8_t const video_timing_pre_pll_clock_div = 0x03; // 1, 2 or 3 depending on clock_frequency
    static uint8_t const video_timing_pixel_clock_div = 0x05; // 4, 5, 8 or 10
    static uint8_t const video_timing_system_clock_div = 0x01; // must be 1
    static uint16_t const video_timing_clock_mul = 0x2B;

    // mipi_clk = 680 MHz (valid range: 200-916)
    // out_clk = 68 MHz (valid range: 20-114.5)
    // output rate: 170 MB/s
    // pll2_clk = clock_frequency * output_timing_clock_mul / output_timing_pre_pll_clock_div
    // out_clk = pll2_clk / output_timing_system_clock_div / output_timing_pixel_clock_div
    // mipi_clk = pll2_clk
    static uint8_t const output_timing_pre_pll_clock_div = 0x03; // 1, 2 or 3 depending on clock_frequency
    static uint8_t const output_timing_pixel_clock_div = 0x0A; // 8 or 10
    static uint8_t const output_timing_system_clock_div = 0x01; // must be 1 (= 1/2?)
    static uint16_t const output_timing_clock_mul = 0x55;

    // the times 2 because there are 2 lanes in the pipeline, so the time for 1 pixel is half the time for 1 pix_clk tick
    static auto const pixel_clock = ch::duration<float>(1.0 / (1e6 * clock_frequency / video_timing_pre_pll_clock_div * video_timing_clock_mul / video_timing_pixel_clock_div * 2));

    static uint16_t const streaming_control_address = 0x0100;
    static uint16_t const frame_bank_switch_address = 0x0150;

    static uint16_t const register_bank_offset[] = {0x0000, 0x0100};

    static uint16_t const minimum_line_blanking = 0x00A8;
    static uint16_t const minimum_line_length = 0x0D78;
    static uint16_t const maximun_line_length = 0x7FF0;
    static uint16_t const minimum_frame_blanking = 0x0020;
    static uint16_t const minimum_frame_length = 0x0100;
    static uint16_t const maximum_frame_length = 0xFFFE;
  }

  int get_binning_factor(uint8_t v)
  {
    switch(v)
    {
    case 0:
      return 1;
    case 1:
    case 3:
      return 2;
    case 2:
      return 4;
    default:
      throw std::invalid_argument("invalid binning mode");
    }
  }

  enum private_camera_register
  {
    COARSE_INTEGRATION_HI = CAMERA_REGISTER_COUNT
  , COARSE_INTEGRATION_LO

    // @@@TODO: used for Binning Multiplexed Exposure HDR? how does it work?
  , ANALOGUE_GAIN_SHORT
  , COARSE_INTEGRATION_TIME_SHORT_HI
  , COARSE_INTEGRATION_TIME_SHORT_LO

  , ENABLE_COMP10TO8 // 0: raw10, 1: raw10to8

    // number of frames to keep this register set active
    // before switching back
    // might be useful when regularly switching settings,
    // e.g. for multi exposure HDR in low light conditions
  , FRAME_DURATION

    // frame length in lines (2 lines in analogue binning mode)
  , FRAME_LENGTH_HI
  , FRAME_LENGTH_LO
    // frame line length in pixel
  , LINE_LENGTH_HI
  , LINE_LENGTH_LO

    // cropping area in lines (y)/pixels (x)
  , LEFT_HI
  , LEFT_LO
  , RIGHT_HI
  , RIGHT_LO
  , TOP_HI
  , TOP_LO
  , BOTTOM_HI
  , BOTTOM_LO

    // output image dimension
  , WIDTH_HI
  , WIDTH_LO
  , HEIGHT_HI
  , HEIGHT_LO

    // @@@TODO: find out what these actually do; valid values: 1, 3
  , X_ODD_INC
  , Y_ODD_INC

    // lens shading correction control
    // we want as little "magic" (a.k.a. runing things for us) as possible applied by the imager,
    // so simply keep it off
  , LSC_ENABLE
  , LSC_COLOR_MODE
  , LSC_SELECT_TABLE
  , LSC_TUNING_TABLE
  , LSC_WHITE_BALANCE_RG_HI
  , LSC_WHITE_BALANCE_RG_LO
  , LSC_TUNING_COEFF_R
  , LSC_TUNING_COEFF_GR
  , LSC_TUNING_COEFF_GB
  , LSC_TUNING_COEFF_B
  , LSC_TUNING_R_HI
  , LSC_TUNING_R_LO
  , LSC_TUNING_GR_HI
  , LSC_TUNING_GR_LO
  , LSC_TUNING_GB_HI
  , LSC_TUNING_GB_LO
  , LSC_TUNING_B_HI
  , LSC_TUNING_B_LO
  , LSC_KNOT_POINT_FORMAT

  , CSI_DATA_FORMAT_HI
  , CSI_DATA_FORMAT_LO

  , PRIVATE_CAMERA_REGISTER_COUNT
  };

  uint32_t next_multiple(uint32_t v, uint32_t m)
  {
    return ((v + (m - 1)) & ~(m - 1));
  }

  bool die(char const *msg)
  {
    throw std::runtime_error(msg);
  }

  bool die(std::string const &msg)
  {
    throw std::runtime_error(msg);
  }
}

camera_control::camera_control(char const *i2c_device, uint8_t i2c_address, int32_t camera)
  : fd(::open(i2c_device, O_RDWR))
  , i2c_address(i2c_address)
  , registers
    {
      // user settable
      {0x0157, 0x80} // ANALOGUE_GAIN

    , {0x0158, 0x01} // DIGITAL_GAIN_HI
    , {0x0159, 0x00} // DIGITAL_GAIN_LO

    , {0x015D, 0x00} // FLIP_IMAGE

    , {0x0174, 0x00} // BINNING_MODE_H
    , {0x0175, 0x00} // BINNING_MODE_V
    , {0x0176, 0x00} // BINNING_CALC_MODE_H
    , {0x0177, 0x00} // BINNING_CALC_MODE_V

      // internal
    , {0x015A, 0x03} // COARSE_INTEGRATION_HI
    , {0x015B, 0xE8} // COARSE_INTEGRATION_LO

    , {0x0189, 0x00} // ANALOGUE_GAIN_SHORT
    , {0x018A, 0x01} // COARSE_INTEGRATION_TIME_SHORT_HI
    , {0x018B, 0xF4} // COARSE_INTEGRATION_TIME_SHORT_LO

    , {0x0155, 0x00} // ENABLE_COMP10TO8

    , {0x0154, 0x00} // FRAME_DURATION

    , {0x0160, 0x0A} // FRAME_LENGTH_HI
    , {0x0161, 0xA8} // FRAME_LENGTH_LO
    , {0x0162, 0x0D} // LINE_LENGTH_HI
    , {0x0163, 0x78} // LINE_LENGTH_LO

    , {0x0164, 0x00} // LEFT_HI
    , {0x0165, 0x00} // LEFT_LO
    , {0x0166, 0x0C} // RIGHT_HI
    , {0x0167, 0xCF} // RIGHT_LO
    , {0x0168, 0x00} // TOP_HI
    , {0x0169, 0x00} // TOP_LO
    , {0x016A, 0x09} // BOTTOM_HI
    , {0x016B, 0x9F} // BOTTOM_LO
    , {0x016C, 0x0C} // WIDTH_HI
    , {0x016D, 0xD0} // WIDTH_LO
    , {0x016E, 0x09} // HEIGHT_HI
    , {0x016F, 0xA0} // HEIGHT_LO

    , {0x0170, 0x01} // X_ODD_INC
    , {0x0171, 0x01} // Y_ODD_INC

    , {0x0190, 0x00} // LSC_ENABLE
    , {0x0191, 0x00} // LSC_COLOR_MODE
    , {0x0192, 0x00} // LSC_SELECT_TABLE
    , {0x0193, 0x00} // LSC_TUNING_TABLE
    , {0x0194, 0x00} // LSC_WHITE_BALANCE_RG_HI
    , {0x0195, 0x00} // LSC_WHITE_BALANCE_RG_LO
    , {0x0198, 0x00} // LSC_TUNING_COEFF_R
    , {0x0199, 0x00} // LSC_TUNING_COEFF_GR
    , {0x019A, 0x00} // LSC_TUNING_COEFF_GB
    , {0x019B, 0x00} // LSC_TUNING_COEFF_B
    , {0x019C, 0x00} // LSC_TUNING_R_HI
    , {0x019D, 0x00} // LSC_TUNING_R_LO
    , {0x019E, 0x00} // LSC_TUNING_GR_HI
    , {0x019F, 0x00} // LSC_TUNING_GR_LO
    , {0x01A0, 0x00} // LSC_TUNING_GB_HI
    , {0x01A1, 0x00} // LSC_TUNING_GB_LO
    , {0x01A2, 0x00} // LSC_TUNING_B_HI
    , {0x01A3, 0x00} // LSC_TUNING_B_LO
    , {0x01A4, 0x00} // LSC_KNOT_POINT_FORMAT

    , {0x018C, 0x0A} // CSI_DATA_FORMAT_HI
    , {0x018D, 0x0A} // CSI_DATA_FORMAT_LO
    }
  , rawcam()
  , pool()
  , target_frame_time(std::experimental::in_place, 1.0 / 10.0)
  , coarse_integration_time(75e-3)
{
  static_assert(register_count == PRIVATE_CAMERA_REGISTER_COUNT, "register_count doesn't match number of registers in enums!");

  fd || die("failed to open i2c device");
  auto linux_error = ioctl(fd, I2C_SLAVE_FORCE, i2c_address);
  (linux_error >= 0) || die("failed to set i2c target");

  if(constants::model_id != read_i2c<uint16_t>(constants::model_id_address))
  {
    die("couldn't find an imx219 imager at i2c address " + std::to_string(i2c_address));
  }

  // ensure the imager isn't running while we're setting everything up
  stop_streaming();

  // setup mmal
  auto error_code = MMAL_SUCCESS;

  error_code = mmal_component_create("vc.ril.rawcam", &rawcam);
  (MMAL_SUCCESS == error_code) || die("failed to create MMAL rawcam");
  RASPI_ON_FAILURE(this) { mmal_component_destroy(rawcam); };

  auto output = rawcam->output[0];

  if(camera > 0)
  {
    error_code = mmal_port_parameter_set_int32(output, MMAL_PARAMETER_CAMERA_NUM, camera);
    (MMAL_SUCCESS == error_code) || die("failed to set MMAL rawcam camera number");
  }

  MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_config = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_config)}};
  error_code = mmal_port_parameter_get(output, &rx_config.hdr);
  (MMAL_SUCCESS == error_code) || die("failed to get parameters for rawcam output");

  rx_config.data_lanes = 2;
  rx_config.image_id = 0x2B;

  error_code = mmal_port_parameter_set(output, &rx_config.hdr);
  (MMAL_SUCCESS == error_code) || die("failed to set parameters for rawcam output");

  error_code = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  (MMAL_SUCCESS == error_code) || die("failed to enable zero copy on MMAL rawcam output");

  set_roi({0, 0, constants::imager_width, constants::imager_height});
  set_output_mode(camera_output_mode::bayer10);

  pool = mmal_port_pool_create(output, output->buffer_num, output->buffer_size);
  pool || die("failed to enable MMAL rawcam output");
  RASPI_ON_FAILURE(this, output) { mmal_port_pool_destroy(output, pool); };

  output->userdata = reinterpret_cast<decltype(output->userdata)>(this);
  error_code = mmal_port_enable(output, callback);
  (MMAL_SUCCESS == error_code) || die("failed to enable MMAL rawcam output");
  RASPI_ON_FAILURE(output) { mmal_port_disable(output); };

  // allow changing manufacturer registers
  write_i2c(0x30EB, uint8_t(0x05));
  write_i2c(0x30EB, uint8_t(0x0C));
  write_i2c(0x300A, uint8_t(0xFF));
  write_i2c(0x300B, uint8_t(0xFF));
  write_i2c(0x30EB, uint8_t(0x05));
  write_i2c(0x30EB, uint8_t(0x09));

  // CSI settings
  write_i2c(0x0114, uint8_t(0x01)); // 2 MIPI lanes

  // global timing
  write_i2c(0x0128, uint8_t(0x00)); // automatic MIPI timing
  write_i2c(0x012A, static_cast<uint16_t>(constants::clock_frequency * 0x100)); // EXCK_FREQ

  // video timing
  write_i2c(0x0304, uint8_t(constants::video_timing_pre_pll_clock_div)); // PREPLLCK_VT_DIV
  write_i2c(0x0301, uint8_t(constants::video_timing_pixel_clock_div)); // VTPXCK_DIV
  write_i2c(0x0303, uint8_t(constants::video_timing_system_clock_div)); // VTSYCK_DIV
  write_i2c(0x0306, uint16_t(constants::video_timing_clock_mul)); // PLL_VT_MPY

  // output timing
  write_i2c(0x0305, uint8_t(constants::output_timing_pre_pll_clock_div)); // PREPLLCK_OP_DIV
  write_i2c(0x0309, uint8_t(constants::output_timing_pixel_clock_div)); // OPPXCK_DIV - default
  write_i2c(0x030B, uint8_t(constants::output_timing_system_clock_div)); // OPSYCK_DIV - default
  write_i2c(0x030C, uint16_t(constants::output_timing_clock_mul)); // PLL_OP_MPY - default: 0x0075; why not use the default?

  // manufacturer specific undocumented magic
  // @@@TODO: do we need this?
  write_i2c(0x455E, uint8_t(0x00));
  write_i2c(0x471E, uint8_t(0x4B));
  write_i2c(0x4767, uint8_t(0x0F));
  write_i2c(0x4750, uint8_t(0x14));
  write_i2c(0x4540, uint8_t(0x00));
  write_i2c(0x47B4, uint8_t(0x14));
  write_i2c(0x4713, uint8_t(0x30));
  write_i2c(0x478B, uint8_t(0x10));
  write_i2c(0x478F, uint8_t(0x10));
  write_i2c(0x4797, uint8_t(0x0E));
  write_i2c(0x479B, uint8_t(0x0E));

  // set default camera settings
  apply_changes();
}

camera_control::~camera_control()
{
  stop_streaming();

  auto output = rawcam->output[0];
  auto error_code = MMAL_SUCCESS;
  error_code = mmal_port_disable(output);
  (MMAL_SUCCESS == error_code) || die("failed to disable MMAL rawcam output");
  mmal_port_pool_destroy(output, pool);
  error_code = mmal_component_disable(rawcam);
  (MMAL_SUCCESS == error_code) || die("failed to disable MMAL rawcam");
  error_code = mmal_component_destroy(rawcam);
  (MMAL_SUCCESS == error_code) || die("failed to destroy MMAL rawcam");
}

void camera_control::apply_changes()
{
  // ensure all changes are atomic by writing them to the currently inactive register bank and
  // then activating that bank

  uint8_t const active_register_bank = read_i2c<uint8_t>(constants::frame_bank_switch_address);
  uint8_t const target_register_bank = !active_register_bank;
  uint16_t const register_offset = constants::register_bank_offset[target_register_bank];

  update_roi();
  update_timing();

  for(auto &&r : registers)
  {
    write_i2c(static_cast<uint16_t>(r.address + register_offset), r.value);
  }

  write_i2c(constants::frame_bank_switch_address, target_register_bank);
}

void camera_control::set_output_mode(camera_output_mode mode)
{
  MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_config = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_config)}};
  auto output = rawcam->output[0];

  auto error_code = mmal_port_parameter_get(output, &rx_config.hdr);
  (MMAL_SUCCESS == error_code) || die("failed to get parameters for rawcam output");

  switch(mode)
  {
  case camera_output_mode::bayer8:
    registers[ENABLE_COMP10TO8].value = 0;
    rx_config.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_10;
    rx_config.pack = MMAL_CAMERA_RX_CONFIG_PACK_8;
    output->format->encoding = MMAL_ENCODING_BAYER_SBGGR8;
    break;

  case camera_output_mode::bayer10to8:
    registers[ENABLE_COMP10TO8].value = 1;
    rx_config.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
    rx_config.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
    output->format->encoding = MMAL_ENCODING_BAYER_SBGGR8;
    break;

  case camera_output_mode::bayer10:
    registers[ENABLE_COMP10TO8].value = 0;
    rx_config.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
    rx_config.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
    output->format->encoding = MMAL_ENCODING_BAYER_SBGGR10P;
    break;

  case camera_output_mode::bayer16:
    registers[ENABLE_COMP10TO8].value = 0;
    rx_config.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_10;
    rx_config.pack = MMAL_CAMERA_RX_CONFIG_PACK_16;
    output->format->encoding = MMAL_ENCODING_BAYER_SBGGR16;
    break;
  }

  error_code = mmal_port_parameter_set(output, &rx_config.hdr);
  (MMAL_SUCCESS == error_code) || die("failed to set parameters for rawcam output");

  error_code = mmal_port_format_commit(output);
  (MMAL_SUCCESS == error_code) || die("failed to commit output format");

  apply_changes();
}

void camera_control::set_roi(camera_roi roi)
{
  // some very rough sanity check here - those are def. invalid
  // it's still possible to specify an invalid roi as we aren't considering binning here
  if(roi.width <= 0 || roi.height <= 0 || roi.x_offset < 0 || roi.y_offset < 0 ||
    (roi.x_offset + roi.width > constants::imager_width) || (roi.y_offset + roi.height) > constants::imager_height)
  {
    throw std::runtime_error("invalid imager roi!");
  }

  auto output = rawcam->output[0];

  output->format->es->video.crop.width = roi.width;
  output->format->es->video.crop.height = roi.height;
  output->format->es->video.width = next_multiple(roi.width, 16);
  output->format->es->video.height = next_multiple(roi.height, 16);

  auto error_code = mmal_port_format_commit(output);
  (MMAL_SUCCESS == error_code) || die("failed to commit output format");

  registers[LEFT_HI].value = static_cast<uint8_t>(roi.x_offset >> 8);
  registers[LEFT_LO].value = static_cast<uint8_t>(roi.x_offset);
  registers[TOP_HI].value = static_cast<uint8_t>(roi.y_offset >> 8);
  registers[TOP_LO].value = static_cast<uint8_t>(roi.y_offset);
  registers[WIDTH_HI].value = static_cast<uint8_t>(roi.width >> 8);
  registers[WIDTH_LO].value = static_cast<uint8_t>(roi.width);
  registers[HEIGHT_HI].value = static_cast<uint8_t>(roi.height >> 8);
  registers[HEIGHT_LO].value = static_cast<uint8_t>(roi.height);
}

void camera_control::set_integration_time(Duration time)
{
  coarse_integration_time = time;
}

void camera_control::set_frame_time(Duration time)
{
  if(time == Duration::zero())
  {
    target_frame_time = {};
  }
  else
  {
    target_frame_time.emplace(time);
  }
}

void camera_control::set_callback(Handler new_handler)
{
  handler = std::move(new_handler);
}

void camera_control::start_streaming()
{
  is_streaming = true;

  auto output = rawcam->output[0];
  for(uint32_t i{}; i < output->buffer_num; ++i)
  {
    auto buffer = mmal_queue_get(pool->queue);
    buffer || die("buffer disappeared from pool!");

    auto error_code = mmal_port_send_buffer(output, buffer);
    (MMAL_SUCCESS == error_code) || die("failed to queue MMAL buffer");
  }

  write_i2c(constants::streaming_control_address, uint8_t(1));
}

void camera_control::stop_streaming()
{
  is_streaming = false;
  write_i2c(constants::streaming_control_address, uint8_t(0));
}

uint8_t &camera_control::operator[](camera_register r)
{
  return registers[r].value;
}

void camera_control::callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  auto self = reinterpret_cast<camera_control *>(port->userdata);

  // @@@TODO: also grab embedded line data; it doesn't have the correct size set, though (it's only 2 lines, but we get the full frame)
  if(self->handler && buffer->length > 0 && !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
  {
    self->handler(reinterpret_cast<char *>(buffer->data), buffer->length);
  }

  if(self->is_streaming)
  {
    buffer->length = 0;
    mmal_port_send_buffer(port, buffer);
  }
  else
  {
    mmal_buffer_header_release(buffer);
  }
}

void camera_control::update_roi()
{
  auto const adjust_roi = [this](int binning, int left_hi, int left_lo, int right_hi, int right_lo, int width_hi, int width_lo, int maximum)
  {
    auto const binning_factor = get_binning_factor(registers[binning].value);
    auto const left = (registers[left_hi].value << 8 | registers[left_lo].value);
    auto const width = (registers[width_hi].value << 8 | registers[width_lo].value);
    auto const right = left + binning_factor * (width - 1);
    if(right >= maximum)
    {
      throw std::invalid_argument("invalid roi!");
    }
    registers[right_hi].value = static_cast<uint8_t>(right >> 8);
    registers[right_lo].value = static_cast<uint8_t>(right);
  };

  adjust_roi(BINNING_MODE_H, LEFT_HI, LEFT_LO, RIGHT_HI, RIGHT_LO, WIDTH_HI, WIDTH_LO, constants::imager_width);
  adjust_roi(BINNING_MODE_V, TOP_HI, TOP_LO, BOTTOM_HI, BOTTOM_LO, HEIGHT_HI, HEIGHT_LO, constants::imager_height);
}

void camera_control::update_timing()
{
  auto const width = (registers[WIDTH_HI].value << 8 | registers[WIDTH_LO].value);
  assert(width <= constants::imager_width);
  auto const height = (registers[HEIGHT_HI].value << 8 | registers[HEIGHT_LO].value);
  assert(height <= constants::imager_height);

  // get minimum timings as imposed by the imager
  uint16_t const frame_length_storage = std::max(static_cast<uint16_t>(height + constants::minimum_frame_blanking), constants::minimum_frame_length);
  uint16_t const line_length_storage = std::max(static_cast<uint16_t>(width + constants::minimum_line_blanking), constants::minimum_line_length);
  auto const minimum_storage_time = constants::pixel_clock * (frame_length_storage * line_length_storage);

  // determine the smallest possible frame time that can be realized with the requested parameters
  auto frame_time = minimum_storage_time;
  {
    // check for invalid exposure times
    if(coarse_integration_time > (constants::pixel_clock * (constants::maximum_frame_length * constants::maximun_line_length)))
    {
      throw std::invalid_argument("specified integration time cannot be realized");
    }

    uint16_t line_length = line_length_storage;
    // check for big exposure times (so long that we have to increase the line length to achieve it)
    if(coarse_integration_time > (constants::pixel_clock * (constants::maximum_frame_length * line_length_storage)))
    {
      // determine the smallest possible line length that allows us to use this integration time
      line_length = static_cast<uint16_t>(std::ceil(coarse_integration_time / (constants::pixel_clock * constants::maximum_frame_length)));
    }

    // frame_length must be at least coarse_integration_time [lines] + 4
    uint16_t const lines = static_cast<uint16_t>(std::ceil((coarse_integration_time / (constants::pixel_clock * line_length))) + 4);

    frame_time = std::max(frame_time, lines * (constants::pixel_clock * line_length));
  }

  // ensure that our minimum required frame time does not exceed the specified desired frame time if any
  // if that's the case we cannot realize this parameter set
  if(target_frame_time)
  {
    if(frame_time > *target_frame_time)
    {
      throw std::invalid_argument("the desired frame rate cannot be achieved with the specified settings!");
    }
    frame_time = *target_frame_time;
  }

  // find a suitable timing for our desired integration time
  auto const target_clocks = std::round(frame_time / constants::pixel_clock);

  // solve min |frame_length * line_length - target_clocks|
  // @@@TODO: find a better algorithm to find some solution that satisfies:
  // * |frame_length * line_length - target_clocks| < epsilon_frame_time
  // * |round(coarse_integration_time / (pixel_clocks * line_length)) - coarse_integration_time| < epsilon_integration_time
  auto min_rem = std::numeric_limits<float>::max();
  uint16_t best_line_length{};
  float best_frame_length{};
  uint16_t const maximum_line_length = std::min(constants::maximun_line_length, static_cast<uint16_t>(target_clocks / frame_length_storage));
  for(uint16_t line_length = line_length_storage; line_length < maximum_line_length; ++line_length)
  {
    auto frame_length = std::round(target_clocks / line_length);
    if(frame_length > constants::maximum_frame_length)
    {
      continue;
    }
    auto const rem = abs(target_clocks - line_length * frame_length);
    if(rem < min_rem)
    {
      best_line_length = line_length;
      best_frame_length = frame_length;
    }
    if(rem == 0.0)
    {
      break;
    }
  }

  auto const line_length = best_line_length;
  auto const frame_length = static_cast<uint16_t>(best_frame_length);
  auto const integration_time = static_cast<uint16_t>(coarse_integration_time / (constants::pixel_clock * line_length));

  registers[FRAME_LENGTH_HI].value = static_cast<uint8_t>(frame_length >> 8);
  registers[FRAME_LENGTH_LO].value = static_cast<uint8_t>(frame_length);
  registers[LINE_LENGTH_HI].value = static_cast<uint8_t>(line_length >> 8);
  registers[LINE_LENGTH_LO].value = static_cast<uint8_t>(line_length);
  registers[COARSE_INTEGRATION_HI].value = static_cast<uint8_t>(integration_time >> 8);
  registers[COARSE_INTEGRATION_LO].value = static_cast<uint8_t>(integration_time);
}

template<typename T>
T camera_control::read_i2c(uint16_t register_address)
{
  static_assert(std::is_trivially_copyable<T>::value);

  T value{};

  i2c_msg messages[]
  {
    {
      i2c_address
    , {}
    , sizeof(register_address)
    , reinterpret_cast<uint8_t *>(&register_address)
    }
  , {
      i2c_address
    , I2C_M_RD
    , sizeof(value)
    , reinterpret_cast<uint8_t *>(&value)
    }
  };

  i2c_rdwr_ioctl_data data
  {
    messages
  , 2
  };

  auto const error_code = ioctl(fd, I2C_RDWR, &data);

  if(error_code != static_cast<int>(data.nmsgs))
  {
    throw std::runtime_error("failed to read from i2c: " + std::to_string(error_code));
  }

  // convert to little endian
  for(auto first = reinterpret_cast<char *>(&value), last = first + (sizeof(value) - 1); first < last; ++first, --last)
  {
    std::iter_swap(first, last);
  }

  return value;
}

template<typename T>
void camera_control::write_i2c(uint16_t register_address, T value)
{
  static_assert(std::is_trivially_copyable<T>::value);

  // convert to big endian
  for(auto first = reinterpret_cast<char *>(&value), last = first + (sizeof(value) - 1); first < last; ++first, --last)
  {
    std::iter_swap(first, last);
  }

  // @@@TODO: why don't we do this via I2C_RDWR, too?
  auto first = reinterpret_cast<uint8_t *>(&value);
  auto const last = first + sizeof(value);
  for(; first != last; ++first, ++register_address)
  {
    uint8_t buffer[3]
    {
      static_cast<uint8_t>(register_address >> 8)
    , static_cast<uint8_t>(register_address)
    , *first
    };
    auto const error_code = write(fd, buffer, sizeof(buffer));
    if(error_code != sizeof(buffer))
    {
      throw std::runtime_error("failed to write to i2c: " + std::to_string(error_code));
    }
  }
}

} // namespace raspi
