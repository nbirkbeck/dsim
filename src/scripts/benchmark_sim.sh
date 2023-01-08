
for level in 1k_cars.textpb big_level.textpb; do
    for i in $(seq 0 10); do
        ./bazel-bin/main --filename $(pwd)/../levels/${level} \
                         --model_dir $(pwd)/../models \
                         --benchmark "sim" --heatmap --logtostderr 2>&1 | grep benchmark_sim
    done
done
