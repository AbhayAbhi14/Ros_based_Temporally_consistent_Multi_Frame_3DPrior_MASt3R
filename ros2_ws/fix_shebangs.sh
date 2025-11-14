PYTHON_PATH=$(which python)
echo "Updating Python shebangs in install/mono_reconstruct to use ${PYTHON_PATH}"

find install/mono_reconstruct/lib/mono_reconstruct/ -type f -exec sed -i "1s|^#!.*|#!${PYTHON_PATH}|" {} \;

echo "Done! All scripts now use ${PYTHON_PATH}"
