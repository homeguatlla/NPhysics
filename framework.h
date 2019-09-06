#pragma once

#include <limits>
#include <glm/gtc/constants.hpp>

#define WIN32_LEAN_AND_MEAN             // Excluir material rara vez utilizado de encabezados de Windows

using real = float;

#undef max
const real MAX_REAL = std::numeric_limits<float>::max();

const real EPSILON = glm::epsilon<float>();