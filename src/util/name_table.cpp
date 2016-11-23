#include "util/name_table.hpp"
#include "util/exception.hpp"
#include "util/simple_logger.hpp"
#include "storage/io.hpp"

#include <algorithm>
#include <fstream>
#include <limits>

#include <boost/filesystem/fstream.hpp>

namespace osrm
{
namespace util
{

NameTable::NameTable(const std::string &filename)
{
    // boost::filesystem::ifstream name_stream(filename, std::ios::binary);

    storage::io::FileReader name_stream_file_reader(filename, storage::io::FileReader::HasNoFingerprint);

    // if (!name_stream)
    //     throw exception("Failed to open " + filename + " for reading.");

    unsigned BLOCK_SIZE = 16;
    bool USE_SHARED_MEMORY = false;
    RangeTable<16, false> m_name_table;
    // name_stream >> m_name_table;


//     template <unsigned BLOCK_SIZE, bool USE_SHARED_MEMORY>
// std::istream &operator>>(std::istream &in, RangeTable<BLOCK_SIZE, USE_SHARED_MEMORY> &table)
// {
    // read number of block
    unsigned number_of_blocks = name_stream_file_reader.ReadElementCount32();
    // name_stream_file.read((char *)&number_of_blocks, sizeof(unsigned));
    // read total length
    // name_stream_file.read((char *)&table.sum_lengths, sizeof(unsigned));
    m_name_table.sum_lengths = name_stream_file_reader.ReadElementCount32();

    m_name_table.block_offsets.resize(number_of_blocks);
    m_name_table.diff_blocks.resize(number_of_blocks);

    // read block offsets
    name_stream_file_reader.ReadInto(m_name_table.block_offsets.data(), number_of_blocks);
    // read blocks
    name_stream_file_reader.ReadInto(m_name_table.diff_blocks.data(), number_of_blocks);
//     return in;
// }

    unsigned number_of_chars = name_stream_file_reader.ReadElementCount32();

    m_names_char_list.resize(number_of_chars + 1); //+1 gives sentinel element
    m_names_char_list.back() = 0;
    if (number_of_chars > 0)
    {
        name_stream_file_reader.ReadInto(&m_names_char_list[0], number_of_chars);
    }
    else
    {
        util::SimpleLogger().Write(logINFO)
            << "list of street names is empty in construction of name table from: \"" << filename
            << "\"";
    }
}

std::string NameTable::GetNameForID(const unsigned name_id) const
{
    if (std::numeric_limits<unsigned>::max() == name_id)
    {
        return "";
    }
    auto range = m_name_table.GetRange(name_id);

    std::string result;
    result.reserve(range.size());
    if (range.begin() != range.end())
    {
        result.resize(range.back() - range.front() + 1);
        std::copy(m_names_char_list.begin() + range.front(),
                  m_names_char_list.begin() + range.back() + 1,
                  result.begin());
    }
    return result;
}

std::string NameTable::GetRefForID(const unsigned name_id) const
{
    // Way string data is stored in blocks based on `name_id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                                      ^     ^
    //                                      [range)
    //                                       ^ name_id + 3
    //
    // `name_id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_REF = 3u;
    return GetNameForID(name_id + OFFSET_REF);
}

std::string NameTable::GetPronunciationForID(const unsigned name_id) const
{
    // Way string data is stored in blocks based on `name_id` as follows:
    //
    // | name | destination | pronunciation | ref |
    //                      ^               ^
    //                      [range)
    //                       ^ name_id + 2
    //
    // `name_id + offset` gives us the range of chars.
    //
    // Offset 0 is name, 1 is destination, 2 is pronunciation, 3 is ref.
    // See datafacades and extractor callbacks for details.
    const constexpr auto OFFSET_PRONUNCIATION = 2u;
    return GetNameForID(name_id + OFFSET_PRONUNCIATION);
}

} // namespace util
} // namespace osrm
