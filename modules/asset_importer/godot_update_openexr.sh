rm -rf ../../thirdparty/openexr
cd ../../thirdparty/
git clone https://github.com/openexr/openexr.git -b v2.3.0 openexr
cd openexr
rm -rf Contrib
rm -rf OpenEXR
rm -rf OpenEXR_Viewers
rm -rf PyIlmBase	
rm -rf cmake
rm -rf .git