/*!
 *
 * @author Qingwen Zhang (qingwen@kth.se)
 * @version 2.0
 * @date 2023-10-21
 * @log 
 * 2024-07-03: fix nan print misalignment. And set longest for all tag width
 * 2023-10-21: support multiple platforms, compile with c++17 in windows, linux and macos
 * 
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// STL
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>
#include <array>

class Timer
{
 private:
	using Duration = std::chrono::duration<double, std::nano>;

 public:
	void start() { time_.push(std::chrono::high_resolution_clock::now()); }

	void stop()
	{
		std::chrono::time_point<std::chrono::high_resolution_clock, Duration> now =
		    std::chrono::high_resolution_clock::now();
		last_ = Duration(now - time_.front()).count();
		time_.pop();

		++samples_;

		double delta = last_ - mean_time_;
		mean_time_ += delta / static_cast<double>(samples_);
		double delta_2 = last_ - mean_time_;
		variance_time_ += delta * delta_2;

		total_time_ += last_;
		min_time_ = std::min(min_time_, last_);
		max_time_ = std::max(max_time_, last_);
	}

	bool isTiming() const { return !time_.empty(); }

	double currentSeconds() const { return currentNanoseconds() / 1000000000; }

	double currentMilliseconds() const { return currentNanoseconds() / 1000000; }

	double currentMicroseconds() const { return currentNanoseconds() / 1000; }

	double currentNanoseconds() const
	{
		return isTiming()
		           ? Duration(std::chrono::high_resolution_clock::now() - time_.front())
		                 .count()
		           : 0;
	}

	double lastSeconds() const { return lastNanoseconds() / 1000000000; }

	double lastMilliseconds() const { return lastNanoseconds() / 1000000; }

	double lastMicroseconds() const { return lastNanoseconds() / 1000; }

	double lastNanoseconds() const { return last_; }

	double totalSeconds() const { return totalNanoseconds() / 1000000000; }

	double totalMilliseconds() const { return totalNanoseconds() / 1000000; }

	double totalMicroseconds() const { return totalNanoseconds() / 1000; }

	double totalNanoseconds() const { return currentNanoseconds() + total_time_; }

	double minSeconds() const
	{
		return numSamples() ? minNanoseconds() / 1000000000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double minMilliseconds() const
	{
		return numSamples() ? minNanoseconds() / 1000000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double minMicroseconds() const
	{
		return numSamples() ? minNanoseconds() / 1000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double minNanoseconds() const
	{
		return numSamples() ? min_time_ : std::numeric_limits<double>::quiet_NaN();
	}

	double maxSeconds() const
	{
		return numSamples() ? maxNanoseconds() / 1000000000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double maxMilliseconds() const
	{
		return numSamples() ? maxNanoseconds() / 1000000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double maxMicroseconds() const
	{
		return numSamples() ? maxNanoseconds() / 1000
		                    : std::numeric_limits<double>::quiet_NaN();
	}

	double maxNanoseconds() const
	{
		return numSamples() ? max_time_ : std::numeric_limits<double>::quiet_NaN();
	}

	double meanSeconds() const { return meanNanoseconds() / 1000000000; }

	double meanMilliseconds() const { return meanNanoseconds() / 1000000; }

	double meanMicroseconds() const { return meanNanoseconds() / 1000; }

	double meanNanoseconds() const { return mean_time_; }

	double varianceSeconds() const
	{
		return 1 < numSamples() ? varianceNanoseconds() / 1000000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double varianceMilliseconds() const
	{
		return 1 < numSamples() ? varianceNanoseconds() / 1000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double varianceMicroseconds() const
	{
		return 1 < numSamples() ? varianceNanoseconds() / 1000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double varianceNanoseconds() const
	{
		return 1 < numSamples() ? sampleVarianceNanoseconds()
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double stdSeconds() const
	{
		return 1 < numSamples() ? stdNanoseconds() / 1000000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double stdMilliseconds() const
	{
		return 1 < numSamples() ? stdNanoseconds() / 1000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double stdMicroseconds() const
	{
		return 1 < numSamples() ? stdNanoseconds() / 1000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double stdNanoseconds() const
	{
		return 1 < numSamples() ? std::sqrt(varianceNanoseconds())
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double sampleVarianceSeconds() const
	{
		return 1 < numSamples() ? sampleVarianceNanoseconds() / 1000000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double sampleVarianceMilliseconds() const
	{
		return 1 < numSamples() ? sampleVarianceNanoseconds() / 1000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double sampleVarianceMicroseconds() const
	{
		return 1 < numSamples() ? sampleVarianceNanoseconds() / 1000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double sampleVarianceNanoseconds() const
	{
		return 1 < numSamples() ? variance_time_ / static_cast<double>(numSamples() - 1)
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double populationVarianceSeconds() const
	{
		return 1 < numSamples() ? populationVarianceNanoseconds() / 1000000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double populationVarianceMilliseconds() const
	{
		return 1 < numSamples() ? populationVarianceNanoseconds() / 1000000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double populationVarianceMicroseconds() const
	{
		return 1 < numSamples() ? populationVarianceNanoseconds() / 1000
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	double populationVarianceNanoseconds() const
	{
		return 1 < numSamples() ? variance_time_ / static_cast<double>(numSamples())
		                        : std::numeric_limits<double>::quiet_NaN();
	}

	std::size_t numSamples() const { return samples_; }

 private:
	std::queue<std::chrono::time_point<std::chrono::high_resolution_clock, Duration>> time_;

	std::size_t samples_ = 0;
	double      last_;
	double      total_time_    = 0.0;
	double      mean_time_     = 0.0;
	double      variance_time_ = 0.0;
	double      min_time_      = std::numeric_limits<double>::max();
	double      max_time_      = std::numeric_limits<double>::lowest();
};

class Timing : public Timer
{
 public:
	Timing() = default;

	Timing(std::string const& tag) : tag_(tag) {}

	Timing(std::string const&                                          tag,
	       std::initializer_list<std::pair<std::size_t const, Timing>> init)
	    : tag_(tag), timer_(init)
	{
	}

	using Timer::start;

	void start(std::string const& tag)
	{
		Timer::start();
		tag_ = tag;
	}

	void start(std::string const& tag, std::string const& color)
	{
		start(tag);
		color_ = color;
	}

	Timing const& operator[](std::size_t num) const { return timer_.at(num); }

	Timing& operator[](std::size_t num) { return timer_[num]; }

	std::string const& tag() const { return tag_; }

	void setTag(std::string const& tag) { tag_ = tag; }

	std::string const& color() const { return color_; }

	void setColor(std::string const& color) { color_ = color; }

	static constexpr char const* resetColor() { return "\033[0m"; }
	static constexpr char const* blackColor() { return "\033[30m"; }
	static constexpr char const* redColor() { return "\033[31m"; }
	static constexpr char const* greenColor() { return "\033[32m"; }
	static constexpr char const* yellowColor() { return "\033[33m"; }
	static constexpr char const* blueColor() { return "\033[34m"; }
	static constexpr char const* magentaColor() { return "\033[35m"; }
	static constexpr char const* cyanColor() { return "\033[36m"; }
	static constexpr char const* whiteColor() { return "\033[37m"; }
	static constexpr char const* boldBlackColor() { return "\033[1m\033[30m"; }
	static constexpr char const* boldRedColor() { return "\033[1m\033[31m"; }
	static constexpr char const* boldGreenColor() { return "\033[1m\033[32m"; }
	static constexpr char const* boldYellowColor() { return "\033[1m\033[33m"; }
	static constexpr char const* boldBlueColor() { return "\033[1m\033[34m"; }
	static constexpr char const* boldMagentaColor() { return "\033[1m\033[35m"; }
	static constexpr char const* boldCyanColor() { return "\033[1m\033[36m"; }
	static constexpr char const* boldWhiteColor() { return "\033[1m\033[37m"; }

	void print(
				std::string title = "Default",
				bool random_colors = false, bool bold = false,
	           std::size_t group_colors_level = std::numeric_limits<std::size_t>::max(),
	           int         precision          = 4) const
	{
		printSeconds(title, random_colors, bold, group_colors_level, precision);
	}

	void printSeconds(
		std::string title = "Default",
	    bool random_colors = false, bool bold = false,
	    std::size_t group_colors_level = std::numeric_limits<std::size_t>::max(),
	    int         precision          = 4) const
	{
		static constexpr std::array<const char*, 7> const RC{redColor(),  greenColor(),   yellowColor(),
		                                     blueColor(), magentaColor(), cyanColor(),
		                                     whiteColor()};

		std::array<std::string, 8> label{"Component", "Total", "Last", "Mean",
		                                 "StDev",     "Min",   "Max",  "Steps"};
		std::array<int, 8>          width{
        std::max(static_cast<int>(label[0].length()), longestTag()),
        std::max(static_cast<int>(label[1].length()), precision + 1 + longestTotal()),
        std::max(static_cast<int>(label[2].length()), precision + 1 + longestLast()),
        std::max(static_cast<int>(label[3].length()), precision + 1 + longestMean()),
        std::max(static_cast<int>(label[4].length()), precision + 1 + longestStd()),
        std::max(static_cast<int>(label[5].length()), precision + 1 + longestMin()),
        std::max(static_cast<int>(label[6].length()), precision + 1 + longestMax()),
        std::max(static_cast<int>(label[7].length()), longestSteps())};

		std::array<int, width.size()> left_pad;
		std::array<int, width.size()> right_pad;

		for (std::size_t i{}; label.size() != i; ++i) {
			left_pad[i]  = std::floor((width[i] - static_cast<int>(label[i].length())) / 2.0);
			right_pad[i] = std::ceil((width[i] - static_cast<int>(label[i].length())) / 2.0);
		}

		printf("%s Timings in seconds (s)\n", title.c_str());
		printf("%*s%s%*s", left_pad[0], "", label[0].c_str(), right_pad[0], "");
		for (std::size_t i{1}; label.size() != i; ++i) {
			printf("\t%*s%s%*s", left_pad[i], "", label[i].c_str(), right_pad[i], "");
		}
		printf("\n");

		double values[] = {totalSeconds(), lastSeconds(), meanSeconds(), stdSeconds(),
		                    minSeconds(),   maxSeconds()};

		printf("%s%s%-*s", bold ? "\033[1m" : "", random_colors ? RC[0] : color().c_str(), width[0], tag().c_str());
		for (std::size_t i=0; i<std::size(values); ++i){
			if (std::isnan(totalSeconds())) {
				printf("\t%*s%s%*s", left_pad[i+1], "", "nan", right_pad[i+1], "");
			} else {
				printf("\t%-*.*f", width[i+1], precision, values[i]);
			}
		}
		printf("\t%-*lu%s\n", width[7], numSamples(), resetColor());

		std::size_t i{};
		printSecondsRecurs(1, i, width, random_colors, bold, group_colors_level,
		                   precision, left_pad, right_pad);
	}

	void printSecondsRecurs(int level, std::size_t& i, const std::array<int, 8>& width,
	                        bool random_colors, bool bold, std::size_t group_colors_level,
	                        int precision, const std::array<int, 8>& left_pad, const std::array<int, 8>& right_pad) const
	{
		static constexpr std::array<const char*, 7> const RC{redColor(),  greenColor(),   yellowColor(),
		                                     blueColor(), magentaColor(), cyanColor(),
		                                     whiteColor()};

		for (auto const& [n, t] : timer_) {
			i += level <= group_colors_level;
			std::string tag = std::string(2 * level, ' ') + std::to_string(n) + ". " + t.tag();
			// printf("%s%s%-*s\t%-*.*f\t%-*.*f\t%-*.*f\t%-*.*f\t%-*.*f\t%-*.*f\t%-*lu%s\n",
			// 	bold ? "\033[1m" : "", 
			// 	random_colors ? RC[i % RC.size()] : t.color().c_str(),
			// 	width[0], tag.c_str(), 
			// 	width[1], precision, t.totalSeconds(), 
			// 	width[2], precision, t.lastSeconds(), 
			// 	width[3], precision, t.meanSeconds(), 
			// 	width[4], precision, t.stdSeconds(), 
			// 	width[5], precision, t.minSeconds(), 
			// 	width[6], precision, t.maxSeconds(), 
			// 	width[7], t.numSamples(),
			// 	resetColor());

			double values[] = {t.totalSeconds(), t.lastSeconds(), t.meanSeconds(), t.stdSeconds(),
		                       t.minSeconds(),   t.maxSeconds()};

			printf("%s%s%-*s", bold ? "\033[1m" : "", random_colors ? RC[i % RC.size()] : t.color().c_str(), width[0], tag.c_str());
			for (std::size_t i=0; i<std::size(values); ++i){
				if (std::isnan(totalSeconds())) {
					printf("\t%*s%s%*s", left_pad[i+1], "", "nan", right_pad[i+1], "");
				} else {
					printf("\t%-*.*f", width[i+1], precision, values[i]);
				}
			}
			printf("\t%-*lu%s\n", width[7], t.numSamples(), resetColor());

			t.printSecondsRecurs(level + 1, i, width, random_colors, bold,
			                     group_colors_level, precision, left_pad, right_pad);
		}
	}

 private:
	int longestTag() const
	{
		int l = tag_.length();
		for (auto const& [n, t] : timer_) {
			l = std::max(l, 4 + static_cast<int>(std::to_string(n).length()) + t.longestTag());
		}
		return l;
	}

	int longestTotal() const
	{
		int l = (std::isnan(static_cast<double>(totalSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(totalSeconds())).length());
		// int l = std::to_string(static_cast<int>(totalSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestTotal());
		}
		return l;
	}

	int longestLast() const
	{
		int l = (std::isnan(static_cast<double>(lastSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(lastSeconds())).length());
		// int l = std::to_string(static_cast<int>(lastSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestLast());
		}
		return l;
	}

	int longestMean() const
	{
		int l = (std::isnan(static_cast<double>(meanSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(meanSeconds())).length());
		// int l = std::to_string(static_cast<int>(meanSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestMean());
		}
		return l;
	}

	int longestStd() const
	{
		int l = (std::isnan(static_cast<double>(stdSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(stdSeconds())).length());
		// int l = std::to_string(static_cast<int>(stdSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestStd());
		}
		return l;
	}

	int longestMin() const
	{
		int l = (std::isnan(static_cast<double>(minSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(minSeconds())).length());
		// int l = std::to_string(static_cast<int>(minSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestMin());
		}
		return l;
	}

	int longestMax() const
	{
		int l = (std::isnan(static_cast<double>(maxSeconds())) ? nan_gap_ : std::to_string(static_cast<int>(maxSeconds())).length());
		// int l = std::to_string(static_cast<int>(maxSeconds())).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestMax());
		}
		return l;
	}

	int longestSteps() const
	{
		int l = (std::isnan(static_cast<double>(numSamples())) ? nan_gap_ : std::to_string(static_cast<int>(numSamples())).length());
		// int l = std::to_string(numSamples()).length();
		for (auto const& [_, t] : timer_) {
			l = std::max(l, t.longestSteps());
		}
		return l;
	}

 private:
	std::map<std::size_t, Timing> timer_;
	std::string                   tag_;
	std::string                   color_;
	std::size_t				   nan_gap_ = 0;
};