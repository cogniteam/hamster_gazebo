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
positions="0,1:0,-1:0,0:1,-1:0,-2"

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
	  roslaunch hamster_sim agent.launch namespace:="agent$robot_id" x:=$x y:=$y 2>&1 &
    pid=$!
    echo $pid >> $pid_file
    if [[ "$index" -eq "0" ]]; then
	      sleep ${SPAWNING_INTERVAL}s
    fi
done

sleep $[ $SIMULATION_LAUNCH_TIME + $SPAWNING_INTERVAL ]s

# Await for exit
echo -e "\e[32m\e[21m================================================================================\e[0m
Press Q to exit..."
any_key='a'
until [ $any_key == 'q' -o $any_key == 'Q' ]; do
   read any_key   
done



# Kill processes
while read pid; 
do
  echo 'Killing process' $pid
  kill -2 $pid
done < $pid_file



# Remove process if file
rm $pid_file
