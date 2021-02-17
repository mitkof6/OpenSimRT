/**
 * @file TypeHelpers.h
 *
 * \brief Type helper structures.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "internal/CommonExports.h"

#include <SimTKcommon.h>
#include <type_traits>
#include <utility>

namespace OpenSimRT {

// SFINAE type trait to detect whether T::const_iterator exists.
struct sfinae_base {
    using yes = char;
    using no = yes[2];
};

// Detect whether T::const_iterator exists
template <typename T> struct has_const_iterator : private sfinae_base {
 private:
    template <typename C> static yes& test(typename C::const_iterator*);
    template <typename C> static no& test(...);

 public:
    static const bool value = sizeof(test<T>(nullptr)) == sizeof(yes);
    using type = T;

    void dummy(); // for GCC to supress -Wctor-dtor-privacy
};

template <typename T> struct has_begin_end : private sfinae_base {
 private:
    template <typename C>
    static yes&
    f(typename std::enable_if<std::is_same<
              decltype(static_cast<typename C::const_iterator (C::*)() const>(
                      &C::begin)),
              typename C::const_iterator (C::*)() const>::value>::type*);

    template <typename C> static no& f(...);

    template <typename C>
    static yes&
    g(typename std::enable_if<
            std::is_same<decltype(static_cast<typename C::const_iterator (
                                          C::*)() const>(&C::end)),
                         typename C::const_iterator (C::*)() const>::value,
            void>::type*);

    template <typename C> static no& g(...);

 public:
    static bool const beg_value = sizeof(f<T>(nullptr)) == sizeof(yes);
    static bool const end_value = sizeof(g<T>(nullptr)) == sizeof(yes);

    void dummy(); // for GCC to supress -Wctor-dtor-privacy
};

// Determine if T is a constainer type.
template <typename T>
struct is_container
        : public std::integral_constant<bool,
                                        has_const_iterator<T>::value &&
                                                has_begin_end<T>::beg_value &&
                                                has_begin_end<T>::end_value> {};
// Determine if type is pair (false)
template <typename> struct is_pair : std::false_type {};
// Determine if type is pair (true)
template <typename T, typename U>
struct is_pair<std::pair<T, U>> : std::true_type {};

// Determine if type is SimTK::Vec<M> (false)
template <typename> struct is_simtk_vec : std::false_type {};
// Determine if type is SimTK::Vec<M> (true)
template <int M, class ELT, int STRIDE>
struct is_simtk_vec<SimTK::Vec<M, ELT, STRIDE>> : std::true_type {};

// Determine if type is SimTK::Vector<M> (false)
template <typename> struct is_simtk_vector : std::false_type {};
// Determine if type is SimTK::Vector<M> (true)
template <class ELT>
struct is_simtk_vector<SimTK::Vector_<ELT>> : std::true_type {};

} // namespace OpenSimRT
