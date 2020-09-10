#pragma once

#include <glbinding/nogl.h>

#include <glbinding/gl/bitfield.h>


namespace gl30ext
{


// import bitfields to namespace
using gl::GL_NONE_BIT;
using gl::GL_PERFQUERY_SINGLE_CONTEXT_INTEL;
using gl::GL_UNUSED_BIT;
using gl::GL_2X_BIT_ATI;
using gl::GL_CONTEXT_CORE_PROFILE_BIT;
using gl::GL_PERFQUERY_GLOBAL_CONTEXT_INTEL;
using gl::GL_RED_BIT_ATI;
using gl::GL_SYNC_FLUSH_COMMANDS_BIT;
using gl::GL_TEXTURE_DEFORMATION_BIT_SGIX;
using gl::GL_TEXTURE_STORAGE_SPARSE_BIT_AMD;
using gl::GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT;
using gl::GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT_EXT;
using gl::GL_VERTEX_SHADER_BIT;
using gl::GL_4X_BIT_ATI;
using gl::GL_COMP_BIT_ATI;
using gl::GL_CONTEXT_COMPATIBILITY_PROFILE_BIT;
using gl::GL_CONTEXT_FLAG_DEBUG_BIT;
using gl::GL_ELEMENT_ARRAY_BARRIER_BIT;
using gl::GL_ELEMENT_ARRAY_BARRIER_BIT_EXT;
using gl::GL_FRAGMENT_SHADER_BIT;
using gl::GL_GEOMETRY_DEFORMATION_BIT_SGIX;
using gl::GL_GREEN_BIT_ATI;
using gl::GL_8X_BIT_ATI;
using gl::GL_BLUE_BIT_ATI;
using gl::GL_CONTEXT_FLAG_ROBUST_ACCESS_BIT;
using gl::GL_CONTEXT_FLAG_ROBUST_ACCESS_BIT_ARB;
using gl::GL_GEOMETRY_SHADER_BIT;
using gl::GL_NEGATE_BIT_ATI;
using gl::GL_UNIFORM_BARRIER_BIT;
using gl::GL_UNIFORM_BARRIER_BIT_EXT;
using gl::GL_VERTEX23_BIT_PGI;
using gl::GL_BIAS_BIT_ATI;
using gl::GL_CONTEXT_FLAG_NO_ERROR_BIT_KHR;
using gl::GL_HALF_BIT_ATI;
using gl::GL_TESS_CONTROL_SHADER_BIT;
using gl::GL_TEXTURE_FETCH_BARRIER_BIT;
using gl::GL_TEXTURE_FETCH_BARRIER_BIT_EXT;
using gl::GL_VERTEX4_BIT_PGI;
using gl::GL_QUARTER_BIT_ATI;
using gl::GL_SHADER_GLOBAL_ACCESS_BARRIER_BIT_NV;
using gl::GL_TESS_EVALUATION_SHADER_BIT;
using gl::GL_COMPUTE_SHADER_BIT;
using gl::GL_EIGHTH_BIT_ATI;
using gl::GL_SHADER_IMAGE_ACCESS_BARRIER_BIT;
using gl::GL_SHADER_IMAGE_ACCESS_BARRIER_BIT_EXT;
using gl::GL_COMMAND_BARRIER_BIT;
using gl::GL_COMMAND_BARRIER_BIT_EXT;
using gl::GL_SATURATE_BIT_ATI;
using gl::GL_PIXEL_BUFFER_BARRIER_BIT;
using gl::GL_PIXEL_BUFFER_BARRIER_BIT_EXT;
using gl::GL_TEXTURE_UPDATE_BARRIER_BIT;
using gl::GL_TEXTURE_UPDATE_BARRIER_BIT_EXT;
using gl::GL_BUFFER_UPDATE_BARRIER_BIT;
using gl::GL_BUFFER_UPDATE_BARRIER_BIT_EXT;
using gl::GL_FRAMEBUFFER_BARRIER_BIT;
using gl::GL_FRAMEBUFFER_BARRIER_BIT_EXT;
using gl::GL_TRANSFORM_FEEDBACK_BARRIER_BIT;
using gl::GL_TRANSFORM_FEEDBACK_BARRIER_BIT_EXT;
using gl::GL_ATOMIC_COUNTER_BARRIER_BIT;
using gl::GL_ATOMIC_COUNTER_BARRIER_BIT_EXT;
using gl::GL_SHADER_STORAGE_BARRIER_BIT;
using gl::GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT;
using gl::GL_COVERAGE_BUFFER_BIT_NV;
using gl::GL_QUERY_BUFFER_BARRIER_BIT;
using gl::GL_COLOR3_BIT_PGI;
using gl::GL_FONT_X_MIN_BOUNDS_BIT_NV;
using gl::GL_COLOR4_BIT_PGI;
using gl::GL_FONT_Y_MIN_BOUNDS_BIT_NV;
using gl::GL_EDGEFLAG_BIT_PGI;
using gl::GL_FONT_X_MAX_BOUNDS_BIT_NV;
using gl::GL_FONT_Y_MAX_BOUNDS_BIT_NV;
using gl::GL_INDEX_BIT_PGI;
using gl::GL_FONT_UNITS_PER_EM_BIT_NV;
using gl::GL_MAT_AMBIENT_BIT_PGI;
using gl::GL_FONT_ASCENDER_BIT_NV;
using gl::GL_MAT_AMBIENT_AND_DIFFUSE_BIT_PGI;
using gl::GL_MAP_PERSISTENT_BIT;
using gl::GL_FONT_DESCENDER_BIT_NV;
using gl::GL_MAT_DIFFUSE_BIT_PGI;
using gl::GL_MAP_COHERENT_BIT;
using gl::GL_FONT_HEIGHT_BIT_NV;
using gl::GL_MAT_EMISSION_BIT_PGI;
using gl::GL_BOLD_BIT_NV;
using gl::GL_GLYPH_WIDTH_BIT_NV;
using gl::GL_DYNAMIC_STORAGE_BIT;
using gl::GL_FONT_MAX_ADVANCE_WIDTH_BIT_NV;
using gl::GL_MAT_COLOR_INDEXES_BIT_PGI;
using gl::GL_GLYPH_HEIGHT_BIT_NV;
using gl::GL_ITALIC_BIT_NV;
using gl::GL_CLIENT_STORAGE_BIT;
using gl::GL_FONT_MAX_ADVANCE_HEIGHT_BIT_NV;
using gl::GL_MAT_SHININESS_BIT_PGI;
using gl::GL_GLYPH_HORIZONTAL_BEARING_X_BIT_NV;
using gl::GL_SPARSE_STORAGE_BIT_ARB;
using gl::GL_FONT_UNDERLINE_POSITION_BIT_NV;
using gl::GL_MAT_SPECULAR_BIT_PGI;
using gl::GL_GLYPH_HORIZONTAL_BEARING_Y_BIT_NV;
using gl::GL_FONT_UNDERLINE_THICKNESS_BIT_NV;
using gl::GL_NORMAL_BIT_PGI;
using gl::GL_GLYPH_HORIZONTAL_BEARING_ADVANCE_BIT_NV;
using gl::GL_GLYPH_HAS_KERNING_BIT_NV;
using gl::GL_FONT_HAS_KERNING_BIT_NV;
using gl::GL_TEXCOORD1_BIT_PGI;
using gl::GL_GLYPH_VERTICAL_BEARING_X_BIT_NV;
using gl::GL_FONT_NUM_GLYPH_INDICES_BIT_NV;
using gl::GL_MULTISAMPLE_BIT_3DFX;
using gl::GL_MULTISAMPLE_BIT_ARB;
using gl::GL_MULTISAMPLE_BIT_EXT;
using gl::GL_TEXCOORD2_BIT_PGI;
using gl::GL_GLYPH_VERTICAL_BEARING_Y_BIT_NV;
using gl::GL_TEXCOORD3_BIT_PGI;
using gl::GL_GLYPH_VERTICAL_BEARING_ADVANCE_BIT_NV;
using gl::GL_TEXCOORD4_BIT_PGI;
using gl::GL_ALL_BARRIER_BITS;
using gl::GL_ALL_BARRIER_BITS_EXT;
using gl::GL_ALL_SHADER_BITS;


} // namespace gl30ext
