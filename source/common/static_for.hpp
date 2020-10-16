/*
 * Copyright (C) 2020 fleroviux
 *
 * Licensed under GPLv3 or any later version.
 * Refer to the included LICENSE file.
 */

#pragma once

#include <utility>

namespace common {

namespace detail {
template <typename T, T Begin,  class Func, T ...Is>
constexpr void static_for_impl( Func &&f, std::integer_sequence<T, Is...> ) {
  ( f( std::integral_constant<T, Begin + Is>{ } ),... );
}
} // namespace detail

template <typename T, T Begin, T End, class Func >
constexpr void static_for( Func &&f ) {
  detail::static_for_impl<T, Begin>( std::forward<Func>(f), std::make_integer_sequence<T, End - Begin>{ } );
}

} // namespace common
