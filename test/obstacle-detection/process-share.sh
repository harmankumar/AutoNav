# Run obstacle test on all captured files in share
for file in `ls -1 ../share | egrep "[0-9]+.jpg"`
do
    echo "../share/"$file".new"
    ./obstacle_test "../share/"$file "../share/new-"$file
done
