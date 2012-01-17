#!/bin/bash

DIR=`readlink -f $1`
if  test ! -d "$DIR"; then  
  echo "'$DIR' is not a directory" 
  echo "Usage: $0 <directory-where-results-are>"
  echo "e.g., $0 some/path/to/SURF/"
  exit
fi
pushd $DIR > /dev/null
for num in 1 ; do 
  echo '"Name";'$DIR';"RMSE (meter)";;;;;;;;;;;;;;"Duration";;"FPS"' > ate_evaluation_$num.csv
  #rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 
  rm -f eval_translational.ate.txt eval_runtime.txt 
  for BASENAME in `ls -d rgbd_dataset_freiburg1_desk2*`; do
    echo $BASENAME
    if test ! -d "$BASENAME";then
      echo "Not ready yet"
      break;
    fi
    ESTIMATE_FILE=$BASENAME/${BASENAME}.bagafter${num}_optimization_estimate.txt
    if test ! -f $ESTIMATE_FILE;then
      echo "No estimate at level " $num 
      continue;
    fi
    EVAL_FILE=$ESTIMATE_FILE.evaluation
#rosrun rgbd_benchmark_tools evaluate_rpe.py --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE; then
    if rosrun rgbd_benchmark_tools evaluate_ate.py --plot $BASENAME/$BASENAME.difference_plot$num.pdf --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE.ate; then
      #rosrun rgbd_benchmark_tools align_and_plot.py --plot $BASENAME/$BASENAME.alignment_plot$num.png --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > /dev/null

      #RMSE
      #echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.txt
      echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.ate.txt
      #echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_rotational.txt
      #grep translational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.txt
      #grep rotational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_rotational.txt
      grep translational_error.rmse $EVAL_FILE.ate |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.ate.txt

      #OVERALL RUNTIME
      STARTTIME=`grep "First RGBD-Data Received" $BASENAME/logfile |head -n1|grep -o '13[0-9]*\.'` #timestamp first relevant action
      ENDTIME=`grep "Finished process" $BASENAME/logfile |grep -o '13[0-9]*\.'` #timestamp first relevant action
      STARTTIME_NSEC=`grep "First RGBD-Data Received" $BASENAME/logfile |head -n1|grep -o '\.[0-9]*'` #timestamp first relevant action
      ENDTIME_NSEC=`grep "Finished with $num" $BASENAME/logfile |grep -o '\.[0-9]*'` #timestamp first relevant action
      echo -n "Start; ${STARTTIME%.}.${STARTTIME_NSEC#.};s; End; ${ENDTIME%.}.${ENDTIME_NSEC#.};s;" >> eval_runtime.txt

      #OPTIMIZER RUNTIME
      OPT_TIME=`grep -B1 "Finished with $num" $BASENAME/logfile |head -n1|grep -o '[0-9.]* s'` #timestamp first relevant action
      echo -n "Optimizer Time; ${OPT_TIME% s};s;" >> eval_runtime.txt


      #NUMBER OF NODES
      NODE_NUM=`grep -B20 "Finished with $num" $BASENAME/logfile |grep "G2O Statistics:"|tail -n1|grep -o '[0-9]* nodes'` #timestamp first relevant action
      EDGE_NUM=`grep -B20 "Finished with $num" $BASENAME/logfile |grep "G2O Statistics:"|tail -n1|grep -o '[0-9]* edges'` #timestamp first relevant action
      echo -n "Number of Nodes/Edges; ${NODE_NUM% nodes};${EDGE_NUM% edges};" >> eval_runtime.txt


      echo >> eval_runtime.txt

      #paste "-d;" eval_rotational.txt eval_translational.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' > evaluation_$num.csv
    else
      echo "Evaluation Failed"
    fi
  done
  paste "-d;" eval_translational.ate.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' >> ate_evaluation_$num.csv
  column '-s;' -t  ate_evaluation_$num.csv
  echo ';;=AVERAGE(C2:C10);;;;;;;;;=AVERAGE(L2:L10);;;=AVERAGE(O2:O10);=AVERAGE(P2:P10);=AVERAGE(Q2:Q10);;=AVERAGE(S2:S10)' >> ate_evaluation_$num.csv
  echo ';;=STDEV(C2:C10);;;;;;;;;=STDEV(L2:L10);;;=STDEV(O2:O10);=STDEV(P2:P10);=STDEV(Q2:Q10);;=STDEV(S2:S10)' >> ate_evaluation_$num.csv
  echo Results stored in $DIR/ate_evaluation_$num.csv
  echo ATE Results at Level $num
  #echo RPE Results at Level $num
  #column '-s;' -t  evaluation_$num.csv
done

rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 
popd > /dev/null
