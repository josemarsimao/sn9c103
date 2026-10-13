#! /bin/sh
# unload module
echo " "
echo "****************************"
echo "***** UNLOADING MODULE *****"
echo "****************************"
echo " "
sudo modprobe -r gspca_elgin_sn9c103 2>/dev/null   # o módulo antigo usa o mesmo ID USB
sudo modprobe -r gspca_elgin_fps
sudo modprobe -r gspca_sonixb 2>/dev/null         # driver original do kernel (mesmo ID USB)

# compile module
echo " "
echo "****************************"
echo "***** COMPILING MODULE *****"
echo "****************************"
echo " "
make ELGIN_BUILD=$(date +%Y%m%d%H%M) || { echo "ERRO NA COMPILACAO: modulo nao instalado"; exit 1; }   # compila como usuario (sem sudo); se falhar, para aqui

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
sudo modprobe gspca_elgin_fps

echo "done"
