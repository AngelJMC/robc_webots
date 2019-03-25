
#include "heuristics_algorithm.h"
#include <stdio.h>

static int count = 1;

int heuristics_loadParam(){
  printf("Counts %d", count);
  count++;
  return count;
}

void heuristics_update(){
}