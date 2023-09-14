# run the whole framework
# data_list is [ispd18_test1, ispd18_test2, ispd18_test5]
# data_list=("ispd18_test1" "ispd18_test2" "ispd18_test5")
# data_list=("ispd18_test2" "ispd18_test5")
# data_list=("ispd19_test2")
data_list=("ispd18_test5")
benchmark_path="/home/scratch.rliang_hardware/wli1/cu-gr/benchmark"
for data in "${data_list[@]}"
do
    # run the whole framework
    echo "data: $data"
    # ./drcu  -lef $benchmark_path/$data/$data.input.lef -def $benchmark_path/$data/$data.input.def -thread 8 -guide $benchmark_path/$data/$data.output2 --output ./result/CUGR2_$data.txt --tat 2000000000 > ./log/detail_CUGR2_$data.log
    ./drcu  -lef $benchmark_path/$data/$data.input.lef -def $benchmark_path/$data/$data.input.def -thread 8 -guide $benchmark_path/$data/$data.output6 --output ./result/Ours_$data.txt --tat 2000000000 > ./log/detail_Ours_$data\_T.log
done

