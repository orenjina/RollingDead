#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#define TYPES 4
#define FUNCS 4
#define PRI 0.7
#define SEC 0.3

// fruit[int][int] double :
// An array containing each fruit type. The first index
// is based on the color of the fruit, by some deterministic
// way of organizing. The second index is based on the
// function of that color of the fruit. After accessing
// both indices, the data structure return the probability
// of that color of fruit resulting in that property.
double fruit[TYPES][FUNCS];
int fruitob[TYPES][FUNCS];

void init();
void compute();

int main() {
  init();
}

// Initialize the fruit double array with uniform probability
// Initialize the fruitob double array with 0s
void init() {
  for (int i = 0; i < TYPES; i++) {
    for (int j = 0; j < FUNCS; j++) {
      fruit[i][j] = 1 / FUNCS;
      fruitob[i][j] = 0;
    }
  }
}

// Update the observation table
void update(int type, int func) {
  fruitob[type][func] += 1;
  compute();
}

// Compute the probabilities according to observations
void compute() {
  for (int i = 0; i < TYPES; i++) {
    int types = 0;
    // Recrod which entries are non-zero
    int a = -1;
    int b = -1;
    for (int j = 0; j < FUNCS; j++) {
      if (fruitob[i][j] > 0) {
        if (a >= 0) {
          b = j;
        } else {
          a = j;
        }
        types++;
      }
    }
    if (types == 0) {
      // No data for this section, uniform distribution
      for (int j = 0; j < FUNCS; j++) {
        fruit[i][j] = 1 / FUNCS;
      }
    } else if (types == 1) {
      int p = fruitob[i][a];
      // Data should be recorded in a
      for (int j = 0; j < FUNCS; j++) {
        if (j == a) {
          double num = pow(PRI, (p + 1)) + pow(SEC, (p + 1));
          double den = pow(PRI, p) + pow(SEC, p);
          fruit[i][j] = num / den;
        } else {
          double num = pow(PRI, p) * SEC  + pow(SEC, p) * PRI;
          double den = pow(PRI, p) + pow(SEC, p);
          // Divide by number of things distributed in
          fruit[i][j] = num / (FUNCS - 1) / den;
        }
      }
    } else if (types == 2) {
      // 2 effects confirmed, other effects cannot happen
      int c1 = fruitob[i][a];
      int c2 = fruitob[i][b];
      for (int j = 0; j < FUNCS; j++) {
        // Probability a is primary
        double a1 = pow(PRI, c1) * pow(SEC, c2);
        double b1 = pow(PRI, c2) * pow(SEC, c1);
        double den = a1 + b1;
        if (j == a) {
          fruit[i][j] = a1 / den;
        } else if (j == b) {
          fruit[i][j] = b1 / den;
        } else {
          fruit[i][j] = 0;
        }
      }
    } else {
      // This program is wrong or the instructors
      // are trolling us
      printf("PANIC");
    }
  }
}
