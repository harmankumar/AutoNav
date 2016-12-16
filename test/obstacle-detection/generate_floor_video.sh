for file in ./floor-video/images/{.,}*;
do
# file='./floor-video/images/0001.jpg'
infile=$file;
outfile="${infile/images/images-processed}";
./obstacle_test $infile $outfile | grep 'angle' >> angle.txt
echo "$outfile";
done
