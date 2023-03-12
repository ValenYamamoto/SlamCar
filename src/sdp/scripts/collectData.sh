for TEST in {1..8} ; do
	FILENAME=stats_output/test$TEST.out
	for SEED in {1..50} ; do
		python3 runForAccuracy.py -y test_params/stats_case_$TEST.yaml -n $SEED -s >> $FILENAME
	done
done
