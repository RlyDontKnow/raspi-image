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

#include <exception>
#include <utility>
#include <type_traits>

#define RASPI_CAT_EXPAND(a, b) a ## b
#define RASPI_CAT(a, b) RASPI_CAT_EXPAND(a, b)
#define RASPI_ANONYMOUS_VAR RASPI_CAT(raspi_anonymous_var_, __COUNTER__)

namespace raspi
{

namespace detail
{
  template<typename T>
  struct scope_value;
}

template<typename F>
struct scope_guard : detail::scope_value<F>::type
{
  explicit scope_guard(F &&fn)
    : scope_value({std::forward<F>(fn)})
  {
  }

  ~scope_guard()
  {
    (*this)();
  }

private:
  using scope_value = typename detail::scope_value<F>::type;
};

template<typename F, bool fire_on_exception = true>
struct scope_guard_exception : detail::scope_value<F>::type
{
  explicit scope_guard_exception(F &&fn)
    : scope_value({std::forward<F>(fn)})
    , initial_exception_count(std::uncaught_exceptions())
  {
  }

  ~scope_guard_exception()
  {
    auto const is_exception_in_flight = std::uncaught_exceptions() != initial_exception_count;
    if(is_exception_in_flight == fire_on_exception)
    {
      (*this)();
    }
  }

private:
  using scope_value = typename detail::scope_value<F>::type;
  int initial_exception_count;
};

namespace detail
{
  template<typename T>
  struct wrapper
  {
    T functor;

    void operator()() noexcept(std::declval<T>()())
    {
      return functor();
    }
  };

  template<typename T>
  struct is_function_pointer
    : std::conditional<std::is_pointer<T>::value,
    std::is_function<typename std::remove_pointer<T>::type>,
    std::is_function<T >> ::type
  {
  };

  template<typename T>
  struct scope_value
  {
    using type = typename std::conditional<is_function_pointer<T>::value, wrapper<T>, T>::type;
  };

  enum class scope_guard_tag
  {
  };

  template<typename F>
  ::raspi::scope_guard<typename std::decay<F>::type>
  operator+(scope_guard_tag, F &&f)
  {
    return ::raspi::scope_guard<typename std::decay<F>::type>(std::forward<F>(f));
  }

  enum class scope_fail_tag
  {
  };

  template<typename F>
  ::raspi::scope_guard_exception<typename std::decay<F>::type, true>
    operator+(scope_fail_tag, F &&f)
  {
    return ::raspi::scope_guard_exception<typename std::decay<F>::type, true>(std::forward<F>(f));
  }

  enum class scope_success_tag
  {
  };

  template<typename F>
  ::raspi::scope_guard_exception<typename std::decay<F>::type, false>
    operator+(scope_success_tag, F &&f)
  {
    return ::raspi::scope_guard_exception<typename std::decay<F>::type, false>(std::forward<F>(f));
  }
}

#define RASPI_ON_SCOPE_EXIT(...) \
  auto RASPI_ANONYMOUS_VAR = ::raspi::detail::scope_guard_tag{} + [__VA_ARGS__]() noexcept

#define RASPI_ON_FAILURE(...) \
  auto RASPI_ANONYMOUS_VAR = ::raspi::detail::scope_fail_tag{} + [__VA_ARGS__]() noexcept

#define RASPI_ON_SUCCESS(...) \
  auto RASPI_ANONYMOUS_VAR = ::raspi::detail::scope_success_tag{} + [__VA_ARGS__]()

} // namespace raspi
