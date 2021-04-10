#pragma once

#include "common/pybind11/embed.h"
#include "common/pybind11/pybind11.h"
#include "common/pybind11/eval.h"

namespace py = pybind11;
using namespace py::literals;

py::object opParams_c = py::module_::import("opParams").attr("common/op_params");

// widget to toggle opParams
class opParamControl : public ToggleControl {
  Q_OBJECT

public:
  opParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent = nullptr) : ToggleControl(title, desc, icon, parent) {
    // set initial state from param
    if (opParams_c().get(param.toStdString().c_str())) {
      toggle.togglePosition();
    }
    QObject::connect(this, &ToggleControl::toggleFlipped, [=](int state) {
      bool value = state ? true : false ;
      opParams_c().put(param.toStdString().c_str(), &value, 1);
    });
  }
};