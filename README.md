# RPI-Solderstation
RaspberryPi 2040/2350 Solder &amp; Heat Plate Station With Touch Screen Open Source Project.

开发板的板载Flash为8MB，而默认为16MB，所以需要替换一个文件。
将项目根目录的“generic_rp2350.json”，复制替换到“C:\Users\用户名\.platformio\platforms\raspberrypi\boards”
使用 VSCode 中的 PlatformIO 打开 RP2350 或 HeatPlate 文件夹。
然后 PlatformIO 会自动在 RP2350 或 HeatPlate 文件夹下生成“.pio”文件夹。
因为 GIT 不会自动上传“.pio”文件夹，而其中包含了驱动代码。
需要先删除自动生成的“.pio”文件夹内的“libdeps”文件夹。
然后将RP2350 或 HeatPlate 文件夹中的“libdeps”文件夹复制到“.pio”文件夹中。
