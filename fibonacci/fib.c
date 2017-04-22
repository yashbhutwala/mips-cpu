#include <stdio.h>

extern int puts(const char *str);

// basic non-optimized recusive fibonacci generator
int fibonacci(int n)
{
   if ( n == 0 )
      return 0;
   else if ( n == 1 )
      return 1;
   else
      return ( fibonacci(n-1) + fibonacci(n-2) );
} 

int main(void)
{
    char s[128]= {0};
    int f = fibonacci(1846);
    puts("Result is:");
    puts(itoa(f));
    puts("\n");

}
