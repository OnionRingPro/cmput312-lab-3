# Script to upload the code over SSH
# Make sure sshpass is installed before running this
if [ $# -eq 0 ]; then
  echo "Pass in the robot ip as an argument"
  exit 1
fi

# Extra arguments specify which files should get uploaded
if [ $# -gt 1 ]; then
  for arg in "${@:2}" 
  do
    sshpass -p "maker" scp -r ./$arg robot@$1:~/cmput312-lab-2
  done
  exit 0
fi

sshpass -p "maker" scp -r ./* robot@$1:~/cmput312-lab-2

