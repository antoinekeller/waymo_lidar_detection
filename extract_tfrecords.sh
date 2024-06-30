for filename in tfrecords/training/*; do
	echo ${filename}
	python3 extract_tfrecord.py ${filename} dataset/training/ --front
done

for filename in tfrecords/validation/*; do
	echo ${filename}
	python3 extract_tfrecord.py ${filename} dataset/validation/ --front
done