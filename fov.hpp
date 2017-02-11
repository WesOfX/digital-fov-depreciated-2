#pragma once

#include <array>

namespace dfov{
	template<
		class map_type,
		class position_type,
		class is_obstruction_function
	>
	auto line_of_sight(
		const map_type& map,
		const position_type& a,
		decltype(a) b,
		const is_obstruction_function& is_obstruction
	) noexcept{
		using axis_type = typename position_type::value_type;
		using column_type = typename map_type::value_type;
		constexpr auto map_rows = std::tuple_size<map_type>::value,
		               map_columns = std::tuple_size<column_type>::value;
		static_assert(
			std::is_signed<axis_type>::value,
			"position_type::value_type must be a signed integer type"
		);
		if(a[0] == b[0] && a[1] == b[1]) return true;
		if(
			a[0] < 0 || a[1] < 0 || b[0] < 0 || b[1] < 0
		 || a[0] >= (axis_type)map_rows
		 || a[1] >= (axis_type)map_columns
		 || b[0] >= (axis_type)map_rows
		 || b[1] >= (axis_type)map_columns
		) return false;
		position_type offset = {
			(axis_type)(b[0] - a[0]),
			(axis_type)(b[1] - a[1])
		},
		quadrant = {
			(axis_type)(offset[0] == 0 ? 1 : (abs(offset[0]) / offset[0])),
			(axis_type)(offset[1] == 0 ? 1 : (abs(offset[1]) / offset[1]))
		},
		transformed_offset = offset;
		for(auto& i: transformed_offset) if(i < 0) i = -i;
		bool inverted_axis = false;
		if(transformed_offset[0] < transformed_offset[1]){
			inverted_axis = true;
			std::swap(transformed_offset[0], transformed_offset[1]);
		}
		auto slope = transformed_offset;
		while(slope[1] > 0){
			slope[0] %= slope[1];
			std::swap(slope[0], slope[1]);
		}
		slope[1] = transformed_offset[1] / slope[0];
		slope[0] = transformed_offset[0] / slope[0];
		for(
			axis_type starting_eps = 0;
			starting_eps < slope[0];
			starting_eps++
		){
			auto eps = starting_eps;
			position_type position = {0, 0};
			for(
				axis_type n = 0;
				n < transformed_offset[0];
				n++
			){
				eps += slope[1];
				if(eps >= slope[0]){
					eps -= slope[0];
					position[inverted_axis] += quadrant[inverted_axis];
					position[!inverted_axis] += quadrant[!inverted_axis];
				} else position[inverted_axis] += quadrant[inverted_axis];
				if(
					is_obstruction(
						map[a[0] + position[0]][a[1] + position[1]]
					)
				) break;
			}
			if(offset[inverted_axis] == position[inverted_axis])
				return true;
		}
		return false;
	}
};
