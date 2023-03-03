N=1
SIMULATION=false
JETSON=false
TEST=false
AUTO=false
PID=false
while getopts 'n:sjtap' opt; do
	case "$opt" in
		n)
		  N=$OPTARG
		  ;;
		s)
		  SIMULATION=true
		  ;;
		j)
		  JETSON=true
		  ;;
		t)
		  TEST=true
		  ;;
		a)
		  AUTO=true
		  ;;
		p)
		  PID=true
		  ;;
	esac
done

if $TEST ; then
	TEST_FILENAME="$(pwd)/test_params/test_case_$N.yaml"
	echo $TEST_FILENAME > params_filename.txt

	if $JETSON ; then
		rosrun sdp runTestCase.py -y $TEST_FILENAME
	else
		if $SIMULATION ; then
			python3 runTestCase.py -y $TEST_FILENAME -s
		else
			python3 runTestCase.py -y $TEST_FILENAME 
		fi
	fi
elif $AUTO ; then
	TEST_FILENAME="$(pwd)/test_params/auto_case_$N.yaml"
	echo $TEST_FILENAME > params_filename.txt
	FLAGS=""

	if [[ $N -ne 1 ]] ; then
		FLAGS="-t"
	fi

    if $PID ; then
        FLAGS=$FLAGS" -p"
    fi


	if $JETSON ; then
		rosrun sdp runAutoCase.py -y $TEST_FILENAME $FLAGS
	else
		if $SIMULATION ; then
			python3 runAutoCase.py -y $TEST_FILENAME -s $FLAGS
		else
			python3 runAutoCase.py -y $TEST_FILENAME $FLAGS
		fi
	fi
fi
