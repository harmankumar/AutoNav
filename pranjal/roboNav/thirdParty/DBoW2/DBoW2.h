#ifndef __D_T_DBOW2__
#define __D_T_DBOW2__

/// Includes all the data structures to manage vocabularies and image databases
namespace DBoW2
{
}

#include "TemplatedVocabulary.h"
#include "TemplatedDatabase.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "QueryResults.h"
#include "FORB.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> 
  FORBVocabulary;

typedef DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB> 
  FORBDatabase;

#endif

