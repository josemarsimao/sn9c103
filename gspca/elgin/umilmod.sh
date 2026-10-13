#! /bin/sh
# unload module
echo " "
echo "****************************"
echo "***** UNLOADING MODULE *****"
echo "****************************"
echo " "
sudo modprobe -r gspca_elgin_sn9c103

# compile module
echo " "
echo "****************************"
echo "***** COMPILING MODULE *****"
echo "****************************"
echo " "
sudo make

# install module
echo " "
echo "*****************************"
echo "***** INSTALLING MODULE *****"
echo "*****************************"
echo " "
sudo make install

# update module dependencies
echo " "
echo "****************************************"
echo "***** UPDATING MODULE DEPENDENCIES *****"
echo "****************************************"
echo " "
sudo depmod

# load new module
echo " "
echo "******************************"
echo "***** LOADING NEW MODULE *****"
echo "******************************"
echo " "
sudo modprobe gspca_elgin_sn9c103

/media/josemar/DEV/PROJECTS/multiview/bin/Release/Multiview
