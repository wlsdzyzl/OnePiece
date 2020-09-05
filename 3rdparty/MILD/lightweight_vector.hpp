/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/


#pragma once
/*
* Copyright (c) 2013 CodeGuro <CodeGuro@gmail.com>
*
* Permission to use, copy, modify, and distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
namespace gstd
{
	typedef unsigned size_t;
	template < typename T, typename SIZE_T = size_t > class lightweight_vector;
	template < typename T, typename SIZE_T = size_t >
	class _lightweight_vector_iterator
	{
		friend class lightweight_vector< T, SIZE_T >;
	private:
		lightweight_vector< T > * _myvec;
		unsigned _where;
	public:
		_lightweight_vector_iterator() : _myvec(0), _where(0)
		{
		}
		_lightweight_vector_iterator(lightweight_vector< T > * const vec, SIZE_T index) : _myvec(vec), _where(index)
		{
		}
		_lightweight_vector_iterator(_lightweight_vector_iterator const & source) : _myvec(source._myvec), _where(source._where)
		{
		}
		_lightweight_vector_iterator operator + (int advances)
		{
			_lightweight_vector_iterator tmp = *this;
			tmp._where += advances;
			return tmp;
		}
		_lightweight_vector_iterator operator ++ ()
		{
			++_where;
			return *this;
		}
		_lightweight_vector_iterator operator ++ (int)
		{
			_lightweight_vector_iterator tmp = *this;
			return --tmp;
		}
		_lightweight_vector_iterator operator - (int devances)
		{
			_lightweight_vector_iterator tmp = *this;
			tmp._where -= devances;
			return tmp;
		}
		_lightweight_vector_iterator operator -- ()
		{
			--_where;
			return *this;
		}
		_lightweight_vector_iterator operator -- (int)
		{
			_lightweight_vector_iterator tmp = *this;
			return --tmp;
		}
		T & operator * ()
		{
			return (*_myvec)[_where];
		}
		T & operator -> ()
		{
			return **this;
		}
	};
	template < typename T, typename SIZE_T >
	class lightweight_vector
	{
	private:
		T * at;
		SIZE_T length;
		SIZE_T cap;

	public:
		typedef _lightweight_vector_iterator< T, SIZE_T > iterator;
		lightweight_vector() : at(0), length(0), cap(0)
		{
		}
		lightweight_vector(lightweight_vector const & source) : at(source.at ? new T[source.cap] : 0), length(source.length), cap(source.cap)
		{
			for (SIZE_T i = 0; i < length; ++i)
				at[i] = source.at[i];
		}
		lightweight_vector & operator = (lightweight_vector const & source)
		{
			if (at) delete[] at;
			at = ((source.cap > 0) ? new T[source.cap] : 0);
			length = source.length;
			cap = source.cap;
			for (unsigned i = 0; i < source.length; ++i)
				at[i] = source.at[i];
			return *this;
		}
		~lightweight_vector()
		{
			if (at) delete[] at;
		}
		SIZE_T size() const
		{
			return length;
		}
		SIZE_T capacity() const
		{
			return cap;
		}
		void expand()
		{
			cap = 4 + cap * 2;
			T * tmp = at;
			at = new T[cap];
			for (SIZE_T i = 0; i < length; ++i)
				at[i] = tmp[i];
			delete[] tmp;
		}
		void clear()
		{
			if (at) delete[] at;
		}
		void clear_light()
		{
			length = 0;
		}
		void push_back(T const & _value)
		{
			if (!at || length == cap)
				expand();
			at[length++] = _value;
		}
		void pop_back()
		{
			if (length) --length;
		}
		iterator insert(iterator const _where, T const & _value)
		{
			insert(at + _where._where, _value);
			return iterator(this, _where._where);
		}
		T & insert(T * offset, T const & _value)
		{
			if (length == cap)
			{
				SIZE_T index = offset - at;
				expand();
				offset = at + index;
			}
			for (T * i = at + length; i > offset; --at)
				*i = *(i - 1);
			*offset = _value;
			++length;
			return *offset;
		}
		T & insert(SIZE_T const offset, T const & _value)
		{
			return insert(at + offset, _value);
		}
		void erase(iterator const _where)
		{
			erase(at + _where._where);
		}
		void erase(iterator const _first, iterator const _last)
		{
			SIZE_T count = _last._where - _first._where;
			length -= count;
			for (T * i = at + _first._where; i < at + length; ++i)
				*i = *(i + count);
		}
		void erase(T * offset)
		{
			--length;
			for (T * i = offset; i < at + length; ++i)
				*i = *(i + 1);
		}
		void erase(SIZE_T const offset)
		{
			erase(at + offset);
		}
		iterator begin()
		{
			return iterator(this, 0);
		}
		iterator end()
		{
			return iterator(this, length);
		}
		T & front()
		{
			return at[0];
		}
		T & back()
		{
			return at[length - 1];
		}
		T & operator [] (SIZE_T index)
		{
			return at[index];
		}
		const T & operator [] (SIZE_T index) const
		{
			return at[index];
		}
	};
}
