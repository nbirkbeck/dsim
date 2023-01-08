for i in ../levels/*.textpb; do
    ./bazel-bin/main --filename $i --model_dir $(pwd)/../models \
                     --benchmark "" --heatmap --logtostderr
    ffmpeg -framerate 60 -i /tmp/im-%04d.jpg -y /tmp/$(basename $i .textpb).mp4;
    convert -delay 15  /tmp/im-[0-3]*[0,2,4,6,8]0.jpg /tmp/$(basename $i .textpb).gif
done
