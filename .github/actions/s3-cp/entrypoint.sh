#!/bin/sh

set -e

mkdir -p ~/.aws
touch ~/.aws/credentials

echo "[default]
aws_access_key_id = ${AWS_ACCESS_KEY_ID}
aws_secret_access_key = ${AWS_SECRET_ACCESS_KEY}" > ~/.aws/credentials

for FILE in ${FILES}
do
    aws s3 cp ${FILE} s3://${AWS_S3_BUCKET}/${DEST} --region ${AWS_REGION} $*
done

rm -rf ~/.aws
