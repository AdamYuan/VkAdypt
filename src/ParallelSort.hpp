#ifndef ADYPT_PARALLELSORT_HPP
#define ADYPT_PARALLELSORT_HPP

// Inspired by https://github.com/baserinia/parallel-sort/blob/master/parasort.h

#include <algorithm>
#include <cinttypes>
#include <future>
#include <iterator>
#include <random>
#include <type_traits>
#include <vector>

#ifndef PARALLEL_SORTER
#define PARALLEL_SORTER std::sort
#endif

template <typename Iter, typename Compare>
inline void ParallelSort(Iter first, Iter last, Compare compare, uint32_t thread_num, uint32_t sample_factor = 100) {
	using T = typename std::iterator_traits<Iter>::value_type;
	const uint32_t kSize = uint32_t(last - first);
	if (kSize <= 1)
		return;
	const uint32_t kPart = kSize / thread_num;
	if (thread_num <= 1 || kPart <= 2) {
		PARALLEL_SORTER(first, last, compare);
		return;
	}
	const uint32_t kSampleNum = thread_num * sample_factor, kMapNum = thread_num * thread_num;
	std::vector<T> ranges(thread_num);
	std::vector<uint32_t> tmp(kMapNum, 0), map(kMapNum, 0), bucket(kMapNum, 0);

	{
		std::minstd_rand gen{std::random_device{}()};
		std::uniform_int_distribution<uint32_t> dis{0, kSize - 1};
		std::vector<T> samples(kSampleNum);
		for (auto &i : samples)
			i = *(first + dis(gen));
		PARALLEL_SORTER(samples.begin(), samples.end(), compare);
		for (uint32_t i = 0; i < thread_num - 1; ++i)
			ranges[i] = samples[(i + 1) * sample_factor];
	}

	{
		std::vector<std::future<void>> futures(thread_num);
		for (uint32_t i = 0; i < thread_num; i++) {
			uint32_t from = i * kPart, to = (i + 1 == thread_num) ? kSize : (i + 1) * kPart;
			futures[i] =
			    std::async(std::launch::async, [thread_num, i, from, to, &first, &tmp, &bucket, &ranges, &compare]() {
				    uint32_t *local_bucket = bucket.data() + i * thread_num;
				    for (Iter it = first + from; it != first + to; ++it) {
					    uint32_t x;
					    for (x = 0; x < thread_num - 1; ++x) {
						    if (compare(*it, ranges[x])) {
							    ++local_bucket[x];
							    break;
						    }
					    }
					    if (x == thread_num - 1)
						    ++local_bucket[thread_num - 1];
				    }
			    });
		}
	}

	{
		for (uint32_t i = 0; i < kMapNum; ++i)
			tmp[i] = i ? tmp[i - 1] + bucket[((i - 1) % thread_num) * thread_num + (i - 1) / thread_num] : 0;
		for (uint32_t i = 0; i < kMapNum; ++i)
			map[i] = tmp[(i % thread_num) * thread_num + i / thread_num];
		std::copy(map.begin(), map.end(), tmp.begin());
	}

	std::vector<T> sorted(kSize);
	{
		std::vector<std::future<void>> futures(thread_num);
		for (uint32_t i = 0; i < thread_num; i++) {
			uint32_t from = i * kPart, to = (i + 1 == thread_num) ? kSize : (i + 1) * kPart;
			futures[i] =
			    std::async(std::launch::async, [thread_num, i, from, to, &first, &tmp, &sorted, &ranges, &compare]() {
				    uint32_t *local_map = tmp.data() + i * thread_num;
				    for (Iter it = first + from; it != first + to; ++it) {
					    unsigned n;
					    for (n = 0; n < thread_num - 1; n++) {
						    if (compare(*it, ranges[n])) {
							    sorted[local_map[n]++] = *it;
							    break;
						    }
					    }
					    if (n == thread_num - 1)
						    sorted[local_map[thread_num - 1]++] = *it;
				    }
			    });
		}
	}

	{
		std::vector<std::future<void>> futures(thread_num);
		for (uint32_t i = 0; i < thread_num; i++) {
			uint32_t from = map[i], to = (i + 1 == thread_num) ? kSize : map[i + 1];
			futures[i] = std::async(std::launch::async, [from, to, &compare, &sorted]() {
				PARALLEL_SORTER(sorted.data() + from, sorted.data() + to, compare);
			});
		}
	}

	std::copy(sorted.begin(), sorted.end(), first);
}

/* template <typename Iter>
inline void ParallelSort(Iter first, Iter last, uint32_t thread_num, uint32_t sample_factor = 100) {
    ParallelSort(first, last, std::less<typename std::iterator_traits<Iter>::value_type>(), thread_num, sample_factor);
} */

#endif // ADYPT_PARALLELSORT_HPP
