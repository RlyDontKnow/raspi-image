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
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <experimental/optional>
#include <functional>

struct MMAL_COMPONENT_T;
struct MMAL_POOL_T;
struct MMAL_PORT_T;
struct MMAL_BUFFER_HEADER_T;

namespace raspi
{

enum camera_register
{
  // gain = 256 / (256 - X)
  // valid range: [0-232]
  ANALOGUE_GAIN

  // 8.8 fixed point, valid range: 0x0100 - 0x0FFF
, DIGITAL_GAIN_HI
, DIGITAL_GAIN_LO

, FLIP_IMAGE // 0: none, 1: horizontal, 2: vertical, 3: both

, BINNING_MODE_H // 0: none, 1: 2x2, 2: 4x4, 3: 2x2 analogue
, BINNING_MODE_V // 0: none, 1: 2x2, 2: 4x4, 3: 2x2 analogue
, BINNING_CALC_MODE_H // 0: average 1: sum
, BINNING_CALC_MODE_V // 0: average 1: sum

, CAMERA_REGISTER_COUNT
};

enum class camera_output_mode
{
  // 8 upper bits
  bayer8
  // 8 bit compressed output
, bayer10to8
  // 10 bit packed bayer data as produced by the sensor
  // see http://picamera.readthedocs.io/en/release-1.10/recipes2.html#bayer-data
, bayer10
  // same bayer pattern as for bayer 10, but each value is padded with 0 to 16 bit
, bayer16
};

// specify cropping roi within the active imager area
// active area dimensions are 3280 x 2464
struct camera_roi
{
  // top-left roi coordinate relative to active imager area
  uint16_t x_offset;
  uint16_t y_offset;
  // dimensions of the output image
  uint16_t width;
  uint16_t height;
};

class camera_control
{
public:
  using Handler = std::function<void(char *data, size_t size)>;
  using Duration = std::chrono::duration<float>;

  camera_control(char const *i2c_device = "/dev/i2c-0", uint8_t i2c_address = 0x10, int32_t camera = -1);
  ~camera_control();

  void apply_changes(); // swap register sets, so changes will take effect next vsync

  void set_output_mode(camera_output_mode);
  void set_roi(camera_roi);
  void set_integration_time(Duration);
  void set_frame_time(Duration); // set desired frame time; may be zero to indicate "don't care" (we'll go as fast as we can then)
  void set_callback(Handler);

  void start_streaming();
  void stop_streaming();


  // change registers
  uint8_t &operator[](camera_register);

private:
  static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

  void update_roi();
  void update_timing();

  template<typename T>
  void write_i2c(uint16_t register_address, T value);

  template<typename T>
  T read_i2c(uint16_t register_address);


  static int const register_count
    = CAMERA_REGISTER_COUNT // public registers above
    + 46 // private registers
    ;

  struct i2c_register
  {
    uint16_t address;
    uint8_t value;
  };

  int fd;
  uint8_t i2c_address;
  i2c_register registers[register_count];
  uint8_t active_register_bank;
  MMAL_COMPONENT_T *rawcam;
  MMAL_POOL_T *pool;
  std::experimental::optional<Duration> target_frame_time;
  Duration coarse_integration_time;
  std::atomic<bool> is_streaming;
  Handler handler;
};

} // namespace raspi
