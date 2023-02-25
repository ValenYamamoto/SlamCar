N=1
SIMULATION=false
JETSON=false
while getopts 'n:sj' opt; do
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
	esac
done

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

