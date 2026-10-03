/// @file
/// @brief The exception translator declared in common.hpp.
#include "common.hpp"

namespace slamlab::bindings {

void register_error_translation() {
  nanobind::register_exception_translator([](const std::exception_ptr& p, void*) {
    try {
      std::rethrow_exception(p);
    } catch (const CoreError& e) {
      const char* name = "SlamlabError";
      switch (e.kind()) {
        case foundations::ErrorKind::kInvalidInput:
          name = "InvalidInputError";
          break;
        case foundations::ErrorKind::kDegenerate:
          name = "DegenerateError";
          break;
        case foundations::ErrorKind::kNotConverged:
          name = "NotConvergedError";
          break;
      }
      const nanobind::object errors = nanobind::module_::import_("slamlab.foundations.errors");
      PyErr_SetString(errors.attr(name).ptr(), e.what());
    }
  });
}

}  // namespace slamlab::bindings
