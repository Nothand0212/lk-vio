#ifndef LVIO_ORBVOCABULARY_HPP
#define LVIO_ORBVOCABULARY_HPP

#include "DBoW2/DBoW2/FORB.h"
#include "DBoW2/DBoW2/TemplatedVocabulary.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

#endif  // LVIO_ORBVOCABULARY_HPP