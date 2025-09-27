#!/bin/bash
colors_On_Red='\033[41m' 
colors_Green='\033[0;32m'        # Green
colors_On_Green='\033[42m'       # Green
colors_Normal='\e[0m'

failed_and_exit()
{
    echo -e "$colors_On_Red $1 $colors_Normal"
    exit
}
print_err_and_exit_if_failed()
{
    if [ $? != 0 ]
    then
        failed_and_exit "$1"
    fi
}

echo_G()
{
    echo -e "$colors_On_Green $1 $colors_Normal"

}
echo_R()
{
    echo -e "$colors_On_Red $1 $colors_Normal"

}