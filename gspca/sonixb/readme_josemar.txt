O arquivo autogain_functions.c possui a função gspca_expo_autogain que causa problema no drive quando utilizado com a camera Elgin "Microdia PC Camera (SN9C103 + OV7630)".


Solução:

1 - Comentar todo o código da função.

2 - recompilar utilizando 'sudo make'

3 - Instalar utilizando 'sudo make install'  irá substituir os arquivos gspca_main.ko e gspca_sonixb.ko

4 - reorganizar as dependências dos módulos com 'sudo depmod'

5 - limpar utilizando 'sudo make clean'

6 - Remover os modulos carregados que estão em uso:

		> sudo modprobe -r gspca_sonixb
