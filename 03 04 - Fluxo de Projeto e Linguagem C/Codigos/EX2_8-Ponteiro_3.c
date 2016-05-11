#include <stdio.h>

int vetor[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

void func(int *a){

	int i = 0;

	for(i = 0; i <= 9; i++){
		
		int aux = *(vetor + i);
		
		printf("vetor [%i] = %i\n", i, aux);
		
		
	}
	
	
}

int main(){
	
	func(vetor);
}