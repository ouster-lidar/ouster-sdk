/*
 * Copyright (c) 2017, Matias Fontanini
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TINS_IP_REASSEMBLER2_H
#define TINS_IP_REASSEMBLER2_H

#include <tins/ip.h>
#include <tins/ip_address.h>
#include <tins/macros.h>
#include <tins/pdu.h>

#include <chrono>
#include <map>
#include <vector>

namespace Tins {

/**
 * \cond
 */
namespace Internals {
class IPv4Fragment2 {
   public:
    typedef PDU::serialization_type payload_type;

    IPv4Fragment2() : offset_() {}

    template <typename T>
    IPv4Fragment2(T* pdu, uint16_t offset)
        : payload_(pdu->serialize()), offset_(offset) {}

    const payload_type& payload() const { return payload_; }

    uint16_t offset() const { return offset_; }

   private:
    payload_type payload_;
    uint16_t offset_;
};

class IPv4Stream2 {
   public:
    IPv4Stream2();

    std::chrono::microseconds last_timestamp_;

    void add_fragment(const std::chrono::microseconds& timestamp, IP* ip);
    bool is_complete() const;
    PDU* allocate_pdu() const;
    const IP& first_fragment() const;

   private:
    typedef std::vector<IPv4Fragment2> fragments_type;

    uint16_t extract_offset(const IP* ip);
    bool extract_more_frag(const IP* ip);

    fragments_type fragments_;
    size_t received_size_;
    size_t total_size_;
    IP first_fragment_;
    bool received_end_;
};
}  // namespace Internals

/**
 * \endcond
 */

/**
 * \brief Reassembles fragmented IP packets.
 *
 * This class is fairly simple: just feed packets into it using
 * IPv4Reassembler::process. If the return value is IPv4Reassembler::FRAGMENTED,
 * then the packet is fragmented and we haven't yet seen the missing fragments,
 * hence we can't reassemble it. If the function returns either
 * IPv4Reassembler::NOT_FRAGMENTED (meaning the packet wasn't fragmented) or
 * IPv4Reassembler::REASSEMBLED (meaning the packet was fragmented but it's now
 * reassembled), then you can process the packet normally.
 *
 * Simple example:
 *
 * \code
 * IPv4Reassembler reassembler;
 * Sniffer sniffer = ...;
 * sniffer.sniff_loop([&](PDU& pdu) {
 *     // Process it in any case, unless it's fragmented (and can't be
 * reassembled yet) if (reassembler.process(pdu) != IPv4Reassembler::FRAGMENTED)
 * {
 *         // Now actually process the packet
 *         process_packet(pdu);
 *     }
 * });
 * \endcode
 */
class IPv4Reassembler2 {
   public:
    /**
     * The status of each processed packet.
     */
    enum PacketStatus {
        NOT_FRAGMENTED,  ///< The given packet is not fragmented
        FRAGMENTED,      ///< The given packet is fragmented and can't be
                         ///< reassembled yet
        REASSEMBLED  ///< The given packet was fragmented but is now reassembled
    };

    TINS_DEPRECATED(typedef PacketStatus packet_status);

    /**
     * The type used to represent the overlapped segment reassembly
     * technique to be used.
     */
    enum OverlappingTechnique { NONE, REPLACE };

    /**
     * Default constructor
     */
    IPv4Reassembler2();

    /**
     * Constructs an IPV4Reassembler.
     *
     * \param technique The technique to be used for reassembling
     * overlapped fragments.
     */
    IPv4Reassembler2(OverlappingTechnique technique);

    /**
     * \brief Processes a PDU and tries to reassemble it.
     *
     * This method tries to reassemble the provided packet. If
     * the packet is successfully reassembled using previously
     * processed packets, its contents will be modified so that
     * it contains the whole payload and not just a fragment.
     *
     * \param timestamp The timestamp of the packet. Used for timeouts.
     * \param pdu The PDU to process.
     * \return NOT_FRAGMENTED if the PDU does not contain an IP
     * layer or is not fragmented, FRAGMENTED if the packet is
     * fragmented or REASSEMBLED if the packet was fragmented
     * but has now been reassembled.
     */
    PacketStatus process(const std::chrono::microseconds& timestamp, PDU& pdu);

   private:
    typedef std::pair<IPv4Address, IPv4Address> address_pair;
    typedef std::pair<uint16_t, address_pair> key_type;
    typedef std::map<key_type, Internals::IPv4Stream2> streams_type;

    key_type make_key(const IP* ip) const;
    address_pair make_address_pair(IPv4Address addr1, IPv4Address addr2) const;

    streams_type streams_;
    OverlappingTechnique technique_;
};

}  // namespace Tins

#endif  // TINS_IP_REASSEMBLER2_H
