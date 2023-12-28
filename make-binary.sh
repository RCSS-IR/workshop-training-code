#!/bin/sh

mkdir -p binary
cd binary
mkdir -p tmp
cd tmp
rm -rf *
cp ../../build/bin/* .
mkdir lib
cp ~/local/starter/lib/lib* lib/

sed -i 's/LIBPATH=\/usr\/local\/lib/LIBPATH=.\/lib/g' start.sh

cd ..
echo "Please enter the team name:"
read teamname

echo $teamname
mv tmp $teamname

tar -czvf $teamname.tar.gz $teamname
