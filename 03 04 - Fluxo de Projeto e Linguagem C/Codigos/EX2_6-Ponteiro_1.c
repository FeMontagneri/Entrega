#include <stdio.h>
int main()
{


	int count = 10 , *temp , sum = 0 ;
	
	
	temp = &count;                                                              //O ponteiro recebe o endereço da variavel count
	*temp = 20;                                                                 //o conteudo do endereço apontado por temp recebe 20
	temp = &sum;                                                                //O ponteiro recebe o endereço da variavel sum
	*temp = count;                                                              //o conteudo do endereço apontado por temp recebe 20
	printf ("count = %d , *temp = %d , sum = %d \n" , count, *temp, sum);
	
	//int count = 10 , *temp , sum = 0 ;
	
}