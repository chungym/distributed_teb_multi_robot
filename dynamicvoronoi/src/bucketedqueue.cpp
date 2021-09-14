/*
* BSD 3-Clause License
*
* Copyright (c) 2021, chungym
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <dynamicvoronoi/bucketedqueue.h>

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

std::vector<int> BucketPrioQueue::sqrIndices;
int BucketPrioQueue::numBuckets;


BucketPrioQueue::BucketPrioQueue() {
  // make sure the index array is created
  if (sqrIndices.size()==0) initSqrIndices();
  nextBucket = INT_MAX;
    
  // now create the buckets
  //    buckets = std::vector<MyQueue2<INTPOINT> >(numBuckets);
  buckets = std::vector<std::queue<INTPOINT> >(numBuckets);

  // reset element counter 
  count = 0;
}

bool BucketPrioQueue::empty() {
  return (count==0);
}


void BucketPrioQueue::push(int prio, INTPOINT t) {
  if (prio>=(int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  int id = sqrIndices[prio];
  if (id<0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  buckets[id].push(t);
  //    printf("pushing %d with prio %d into %d. Next: %d\n", t.x, prio, id, nextBucket);
  if (id<nextBucket) nextBucket = id;
  //    printf("push new next is %d\n", nextBucket);
  count++;
}

INTPOINT BucketPrioQueue::pop() {
  int i;
  assert(count>0);
  //    printf("next: %d\n", nextBucket);
  for (i = nextBucket; i<(int)buckets.size(); i++) {
    if (!buckets[i].empty()) break;	
  }
  assert(i<(int)buckets.size());
  nextBucket = i;
  //    printf("pop new next %d\n", nextBucket);
  count--;
  INTPOINT p = buckets[i].front();
  buckets[i].pop();
  return p;
}


void BucketPrioQueue::initSqrIndices() {
  //    std::cout << "BUCKETQUEUE Starting to build the index array...\n";
  //  std::set<int> sqrNumbers;

  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {
      int sqr = x*x+y*y;
      sqrIndices[sqr] = count++;
    }
  }
  numBuckets = count;
  //    std::cout << "BUCKETQUEUE Done with building the index arrays.\n";
}
