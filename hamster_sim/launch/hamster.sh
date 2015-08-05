#!/bin/bash

print_help() {
  echo -e "\e[32m================================================================================\e[0m"
  echo "Robots/agents count:"
  echo "simulation_launcher [-r|--robots|-rc|--robotscount]=<robots count>"
  echo ""
  echo "Robots/agents positions:"
  echo "simulation_launcher [-p|--positions|-rp|--robotpositions]=<x1,y1;x2,y2;x3,y3>"
  echo -e "\e[32m================================================================================\e[0m"
}

if [[ $1 == "-h" || $1 == "--help" || $1 == "help" ]]; then
  print_help
  exit 0
fi


clear
SIMULATION_LAUNCH_TIME=10
SPAWNING_INTERVAL=10
robots=2
genpositions()
{
  stderr() {
     echo $@ >&2
  }
  abs() {
    python <<EOF
import sys
print abs(int($1))
EOF
    return $?
  }
  frac() {
    python <<EOF
import sys
print '{0:.2f}'.format(float($1) / float($2))
EOF
    return $?
  }
  max=0
  x=0
  y=0
  r=0
  poses=""
  while [ $r -lt $1 ]; do
    stderr max=$max.x=$x.y=$y
    absx=`abs $x`
    absy=`abs $y`
    if [ $absx -eq $max ] || [ $absy -eq $max ]; then
      fracx=`frac $x 2`
      fracy=`frac $y 2`
      poses="$poses:$fracx,$fracy"
      stderr $x,$y
      r=`expr $r + 1`
    fi
    if [ $x -eq $max ]; then
      if [ $y -eq $max ]; then
        max=`expr $max + 1`
        y=`expr 0 - $max`
      else
        y=`expr $y + 1`
      fi
      x=`expr 0 - $max`
    else
      x=`expr $x + 1`
    fi
  done
  echo $poses | sed 's/^://g'
}
positions=
for i in "$@"
do
    echo $i
  case $i in

    -r=*|--robots=*|-rc=*|--robotcount=*)
    robots="${i#*=}"
    shift
    ;;
    -p=*|--positions=*|-rp=*|--robotpositions=*)
    positions="${i#*=}"
    shift
    ;;
    -s=*)

    ;;
  esac
done
[ ! $positions ] && positions=`genpositions $robots`

echo -e "\e[32m================================================================================\e[1m

 ███╗   ███╗██████╗ ███╗   ███╗
 ████╗ ████║██╔══██╗████╗ ████║
 ██╔████╔██║██████╔╝██╔████╔██║
 ██║╚██╔╝██║██╔══██╗██║╚██╔╝██║
 ██║ ╚═╝ ██║██║  ██║██║ ╚═╝ ██║
 ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝
 ███████╗██╗███╗   ███╗██╗   ██╗██╗      █████╗ ████████╗██╗ ██████╗ ███╗   ██╗
 ██╔════╝██║████╗ ████║██║   ██║██║     ██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║
 ███████╗██║██╔████╔██║██║   ██║██║     ███████║   ██║   ██║██║   ██║██╔██╗ ██║
 ╚════██║██║██║╚██╔╝██║██║   ██║██║     ██╔══██║   ██║   ██║██║   ██║██║╚██╗██║
 ███████║██║██║ ╚═╝ ██║╚██████╔╝███████╗██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║
 ╚══════╝╚═╝╚═╝     ╚═╝ ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝

\e[21m================================================================================\e[0m"

echo -e "   Robots count:     $robots
   Robots positions: $positions"


# Process id file
pid_file=$(tempfile)



# Kill processes
trap "while read pid; 
do
  echo 'Killing process' $pid
  kill -2 $pid
done < $pid_file



# Remove process if file
rm $pid_file

yes | rosnode cleanup" exit

echo -e "\e[32m\e[21m================================================================================\e[0m
1. Launching hamster simulation"
roslaunch hamster_sim hamster.launch robots_count:=$robots &
pid=$!
echo $pid > $pid_file
sleep ${SIMULATION_LAUNCH_TIME}s
# Agents
echo -e "\e[32m\e[21m================================================================================\e[0m
2. Spawning agents"
IFS=':' read -a position <<< "$positions"

for ((index = 0; index < $robots; index++))
do
    robot_id=$[ $index + 1 ]

    IFS=',' read x y <<< "${position[index]}"

    echo -e "   Spawning 'agent$robot_id' at ($x, $y)" 
	  roslaunch hamster_sim agent.launch namespace:="agent$robot_id" x:=$x y:=$y &
    pid=$!
    echo $pid >> $pid_file
    sleep ${SPAWNING_INTERVAL}s
done

sleep $[ $SIMULATION_LAUNCH_TIME + $SPAWNING_INTERVAL ]s

# Await for exit
echo -e "\e[32m\e[21m================================================================================\e[0m
Press Q to exit..."
any_key='a'

#purge any unhandled inputs
read -n 1000 -t 0.1 dontcare
while [ 1 ]; do
  read -n 1 -t 1 any_key
  [ "$any_key" == "q" -o "$any_key" == "Q" ] && break
done
