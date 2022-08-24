/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_INTERNAL_STATIC_VECTOR_H
#define GNSS_CONVERTERS_INTERNAL_STATIC_VECTOR_H

#include <cassert>
#include <cstddef>
#include <iterator>
#include <type_traits>

namespace gnss_converters {

/**
 * Container which much like std::array is fixed sized and does not use any
 * dynamic memory. User can push data into the container and the container will
 * internally grow to at most its fixed capacity size.
 *
 * @tparam T data type held by container
 * @tparam N maximum fixed size capacity of the container
 */
template <typename T, size_t N>
class StaticVector final {
 public:
  using value_type = T;
  using size_type = size_t;
  using difference_type = ptrdiff_t;
  using reference = T &;
  using const_reference = const T &;
  using pointer = T *;
  using const_pointer = const T *;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

 public:
  StaticVector() noexcept : size_(0), capacity_(N), data_() {}

  StaticVector(const StaticVector &other) noexcept = delete;
  StaticVector(StaticVector &&other) noexcept = delete;

  StaticVector &operator=(const StaticVector &other) noexcept = delete;
  StaticVector &operator=(StaticVector &&other) noexcept = delete;

  ~StaticVector() { clear(); }

  /**
   * @return an iterator to the first element
   */
  iterator begin() noexcept { return static_cast<iterator>(data()); }

  /**
   * @return an iterator to the first element
   */
  const_iterator begin() const noexcept { return cbegin(); }

  /**
   * @return an iterator to the first element
   */
  const_iterator cbegin() const noexcept { return const_iterator(data()); }

  /**
   * @return an iterator to the element following the last element
   */
  iterator end() noexcept { return static_cast<iterator>(&data()[size_]); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator end() const noexcept { return cend(); }

  /**
   * @return an iterator to the element following the last element
   */
  const_iterator cend() const noexcept {
    return const_iterator(&data()[size_]);
  }

  /**
   * @return reverse iterator to the first element
   */
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }

  /**
   * @return reverse iterator to the first element
   */
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(crbegin());
  }

  /**
   * @return reverse iterator to the first element
   */
  const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(cend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

  /**
   * @return reverse iterator to the element following the last element
   */
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(crend());
  }

  /**
   * @return reverse iterator to the element following the last element
   */
  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(cbegin());
  }

  /**
   * @return number of elements in the array
   */
  size_type size() const noexcept { return size_; }

  /**
   * @return capacity of the array
   */
  constexpr size_type capacity() const noexcept { return capacity_; }

  /**
   * @return true if the container is empty, false otherwise
   */
  bool empty() const noexcept { return size_ == 0; }

  /**
   * @return true if the container is full, false otherwise
   */
  bool full() const noexcept { return size_ == capacity_; }

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the array size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  reference operator[](size_type index) {
    assert(index < size_);
    return data()[index];
  }

  /**
   * @note this class asserts in developer builds if the index requested is past
   * the array size.
   *
   * @param index index of the element to return
   * @return reference to the requested element
   */
  const_reference operator[](size_type index) const {
    assert(index < size_);
    return data()[index];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the first element
   */
  reference front() {
    assert(!empty());
    return data()[0];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the first element
   */
  const_reference front() const {
    assert(!empty());
    return data()[0];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the last element
   */
  reference back() {
    assert(!empty());
    return data()[size_ - 1];
  }

  /**
   * @note this class asserts in developer builds if the array is empty.
   *
   * @return reference to the last element
   */
  const_reference back() const {
    assert(!empty());
    return data()[size_ - 1];
  }

  /**
   * @return pointer to the underlying array, nullptr if the array is empty
   */
  inline pointer data() noexcept {
    return static_cast<pointer>(static_cast<void *>(data_));
  }

  /**
   * @return pointer to the underlying array, nullptr if the array is empty
   */
  inline const_pointer data() const noexcept {
    return static_cast<const_pointer>(static_cast<const void *>(data_));
  }

  /**
   * Appends the given element value to the end of the container.
   *
   * @param value value of the element to append
   * @return false if the array is already full, otherwise true.
   */
  bool push_back(const value_type &value) noexcept(
      std::is_nothrow_copy_constructible<value_type>().value) {
    if (size_ == capacity_) {
      return false;
    }

    new (&data()[size_]) value_type(value);
    ++size_;

    return true;
  }

  /**
   * Appends the given element value to the end of the container.
   *
   * @param value value of the element to append
   * @return false if the array is already full, otherwise true.
   */
  bool push_back(value_type &&value) noexcept(
      std::is_nothrow_move_constructible<value_type>().value) {
    if (size_ == capacity_) {
      return false;
    }

    new (&data()[size_]) value_type(std::move(value));
    ++size_;

    return true;
  }

  /**
   * Removes the last element of the container.
   *
   * @return false if the array is empty, otherwise true
   */
  bool pop_back() noexcept(std::is_nothrow_destructible<value_type>().value) {
    if (empty()) {
      return false;
    }

    data()[size_ - 1].~value_type();
    --size_;

    return true;
  }

  /**
   * Erases all elements from the array.
   */
  void clear() noexcept(std::is_nothrow_destructible<value_type>().value) {
    while (size_ > 0) {
      data()[size_ - 1].~value_type();
      --size_;
    }
  }

  /**
   * Appends a new element to the end of the container.
   *
   * @tparam Args argument types to forward to the constructor of the element
   * @param args arguments to forward to the constructor of the element
   * @return false if the array is already full, otherwise true
   */
  template <typename... Args>
  bool emplace_back(Args &&...args) noexcept(
      std::is_nothrow_constructible<value_type, Args &&...>().value) {
    if (size_ == capacity_) {
      return false;
    }

    new (&data()[size_]) value_type(std::forward<Args>(args)...);
    ++size_;

    return true;
  }

 private:
  size_t size_;
  const size_t capacity_;
  std::aligned_storage_t<sizeof(T), alignof(T)> data_[N];
};
}  // namespace gnss_converters

#endif  // GNSS_CONVERTERS_INTERNAL_STATIC_VECTOR_H
