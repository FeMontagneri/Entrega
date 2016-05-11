#include <stdio.h>

void swap(int *a, int *b){
	
	int aux;
	aux = *a;
	*a  = *b;
	*b  = aux;
	
}

int main(){
	
	int x = 10;
	int y = 20;
	
	printf("x = %i\ny = %i\n\n", x, y);
	
	swap(&x, &y);
	
	printf("x = %i\ny = %i\n\n", x, y);
	
	


	
}