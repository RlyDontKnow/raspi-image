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

#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>

void handler(char *data, size_t size)
{
  static std::atomic<int> frame{0};
  std::cout << "got frame " << frame << " with size " << size << '\n';
  std::ofstream fs("test_" + std::to_string(frame++) + ".raw", std::ios::binary);
  fs.write(data, size);
}

int main()
{
  raspi::camera_control test;
  test.set_callback(handler);
  test.set_frame_time(raspi::camera_control::Duration::zero()); // no specific target fps - makes playing with exposure easier
  test.set_output_mode(raspi::camera_output_mode::bayer16);
  test.set_integration_time(raspi::camera_control::Duration(0.1));
  test.apply_changes();
  test.start_streaming();

  std::this_thread::sleep_for(std::chrono::seconds(10));
}
