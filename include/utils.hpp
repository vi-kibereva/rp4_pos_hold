#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <sstream>

namespace utils {

template <class... T>
[[noreturn]] inline void throw_errno(int e, T &&...parts) {
  std::ostringstream os;
  os << std::boolalpha;

  // join args with a single space
  auto append = [&](auto &&x) -> void { os << std::forward<decltype(x)>(x); };
  ((append(std::forward<T>(parts)), os << ' '), ...);
  std::string msg = os.str();
  if (!msg.empty() && msg.back() == ' ')
    msg.pop_back(); // trim trailing space

  // append OS error text (e.g., "No such file or directory")
  msg.append(": ").append(std::system_category().message(e));

  throw std::system_error(e, std::system_category(), std::move(msg));
}

} // namespace utils

#endif // !UTILS_HPP
