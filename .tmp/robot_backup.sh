set -e
echo "=== BACKUP OLD FILE ==="
cp -p /home/spark/codes/grasp_generic.py /home/spark/codes/grasp_generic.py.bak_20261122_miss_branch
md5sum /home/spark/codes/grasp_generic.py /home/spark/codes/grasp_generic.py.bak_20261122_miss_branch
echo "=== DONE ==="