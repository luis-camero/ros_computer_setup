# Parse Arguments
cuda_version=""

while getopts v: flag
do
    case "${flag}" in
        v) cuda_version="${OPTARG}";;
    esac
done

# PRINT: cuda_version
IFS='.' read -ra cuda_version_arr <<< "$cuda_version"
echo "CUDA Version: $cuda_version"

# Linux Version
linux_version=$(lsb_release -c -s)

case $linux_version in
    "bionic")
        ubuntu_version="ubuntu1804"
    ;;
    "focal") 
        ubuntu_version="ubuntu2004"
    ;;
    "jammy")
        ubuntu_version="ubuntu2204"
    ;;
    *)
        echo -e "Linux version '$linux_version' is not supported"
esac
# PRINT: ubuntu_version
echo "Ubuntu Version: $ubuntu_version"


# Install Keys
cuda_keyring_file="cuda_keyring_file.deb"
cuda_keyring_url="https://developer.download.nvidia.com/compute/cuda/repos/$ubuntu_version/x86_64/cuda-keyring_1.0-1_all.deb"

echo -e "\nDownloading Keyring File:"
res=$(wget -O $cuda_keyring_file $cuda_keyring_url)

echo -e "\nInstalling Keys:"
res=$(sudo dpkg -i $cuda_keyring_file)
res=$(rm $cuda_keyring_file)
echo -e "...Done"

# Install CUDA
echo -e "\nUpdate:"
sudo apt-get update
if [ "$cuda_version" == "" ];
then
    cuda_package="cuda"
else
    cuda_package="cuda-${cuda_version_arr[0]}-${cuda_version_arr[1]}"
fi
echo -e "\nInstalling CUDA Packages:"
sudo apt-get -y install $cuda_package
