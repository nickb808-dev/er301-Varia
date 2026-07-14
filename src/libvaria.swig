/* libvaria.swig — SWIG interface for the Varia (Variable Bandwidth Filter) Object.
 * Package "varia"; the internal C++ namespace stays `vbf`. */

%module varia_libvaria

%include <od/glue/mod.cpp.swig>

%{
#undef SWIGLUA
#include <VariableBW.h>
#include <SlopeGraphic.h>
#define SWIGLUA
using namespace vbf;
%}

%include <VariableBW.h>
%include <SlopeGraphic.h>
