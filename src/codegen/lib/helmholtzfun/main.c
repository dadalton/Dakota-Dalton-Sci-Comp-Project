/*
** main.c
*/
#include <stdio.h>
#include <stdlib.h>
#include "helmholtzfun.h"
#include "helmholtzfun_initialize.h"
#include "helmholtzfun_terminate.h"

int main()
{
    helmholtzfun_initialize();
    
    printf(helmholtzfun());
    
    helmholtzfun_terminate();
    
    return 0;
}