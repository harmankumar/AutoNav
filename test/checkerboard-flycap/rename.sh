a=1
for i in *.png; do
  prefix="calib"
  new=$prefix$(printf "%02d.png" "$a") #04 pad to length of 4
  echo $new
  mv "$i" "$new"
  let a=a+1
done
