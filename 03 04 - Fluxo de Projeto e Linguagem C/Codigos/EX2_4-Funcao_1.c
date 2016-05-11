#include <stdio.h>

int hcf(int a, int b){
    
    int i   = 0;
    int acc = 0;
    
    for(i = 1; (i<= a) || (i<=b); i++){
        
        if(((a % i) == 0)&&((b % i) == 0)){
            
            acc = i;
        }
  }
    
    return acc;
    
}
    

int main()
{
    int num1 = 15, num2= 55, aux;
    
    /*
    printf("Digitar o numero 1:\n");
    scanf ("%d", &num1);
    printf("\nDigitar o numero 2:\n");
    scanf ("%d", &num2);
    */
    
    aux = hcf(num1, num2);
    
    printf("\n HCF = %i \n", aux);

    return 0;
}