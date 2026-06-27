/* LSOracle: A learning based Oracle for Logic Synthesis

 * MIT License
 * Copyright 2019 Laboratory for Nano Integrated Systems (LNIS)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <set>
#include <regex>
#include <fstream>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <mockturtle/mockturtle.hpp>

#include "algorithms/partitioning/partition_manager.hpp"
#include "algorithms/partitioning/partition_view.hpp"

namespace oracle {

const std::regex valid_id("^[a-zA-Z][_a-zA-Z0-9]*$");
const std::set<std::string> keywords = {
    // IEEE 1800-2017 Annex B
    "accept_on", "alias", "always", "always_comb", "always_ff", "always_latch",
    "and", "assert", "assign", "assume", "automatic", "before",
    "begin", "bind", "bins", "binsof", "bit", "break",
    "buf", "bufif0", "bufif1", "byte", "case", "casex",
    "casez", "cell", "chandle", "checker", "class", "clocking",
    "cmos", "config", "const", "constraint", "context", "continue",
    "cover", "covergroup", "coverpoint", "cross", "deassign", "default",
    "defparam", "design", "disable", "dist", "do", "edge",
    "else", "end", "endcase", "endchecker", "endclass", "endclocking",
    "endconfig", "endfunction", "endgenerate", "endgroup", "endinterface", "endmodule",
    "endpackage", "endprimitive", "endprogram", "endproperty", "endsequence", "endspecify",
    "endtable", "endtask", "enum", "event", "eventually", "expect",
    "export", "extends", "extern", "final", "first_match", "for",
    "force", "foreach", "forever", "fork", "forkjoin", "function",
    "generate", "genvar", "global", "highz0", "highz1", "if",
    "iff", "ifnone", "ignore_bins", "illegal_bins", "implements", "implies",
    "import", "incdir", "include", "initial", "inout", "input",
    "inside", "instance", "int", "integer", "interconnect", "interface",
    "intersect", "join", "join_any", "join_none", "large", "let",
    "liblist", "library", "local", "localparam", "logic", "longint",
    "macromodule", "matches", "medium", "modport", "module", "nand",
    "negedge", "nettype", "new", "nexttime", "nmos", "nor",
    "noshowcancelled", "not", "notif0", "notif1", "null", "or",
    "output", "package", "packed", "parameter", "pmos", "posedge",
    "primitive", "priority", "program", "property", "protected", "pull0",
    "pull1", "pulldown", "pullup", "pulsestyle_ondetect", "pulsestyle_onevent", "pure",
    "rand", "randc", "randcase", "randsequence", "rcmos", "real",
    "realtime", "ref", "reg", "reject_on", "release", "repeat",
    "restrict", "return", "rnmos", "rpmos", "rtran", "rtranif0",
    "rtranif1", "s_always", "s_eventually", "s_nexttime", "s_until", "s_until_with",
    "scalared", "sequence", "shortint", "shortreal", "showcancelled", "signed",
    "small", "soft", "solve", "specify", "specparam", "static",
    "string", "strong", "strong0", "strong1", "struct", "super",
    "supply0", "supply1", "sync_accept_on", "sync_reject_on", "table", "tagged",
    "task", "this", "throughout", "time", "timeprecision", "timeunit",
    "tran", "tranif0", "tranif1", "tri", "tri0", "tri1",
    "triand", "trior", "trireg", "type", "typedef", "union",
    "unique", "unique0", "unsigned", "until", "until_with", "untyped",
    "use", "uwire", "var", "vectored", "virtual", "void",
    "wait", "wait_order", "wand", "weak", "weak0", "weak1",
    "while", "wildcard", "wire", "with", "within", "wor",
    "xnor", "xor",
};

inline std::string escape_id(std::string id)
{
    if (id[0] == '\\') {
	id.erase(0, 1);
    }
    std::smatch match;
    if(!std::regex_search(id, match, valid_id)) {
	return "\\" + id + " ";;
    }
    if (keywords.find(id) != keywords.end()) {
	return "\\" + id + " ";
    }
    return id;
}

/*! \brief Writes the top-level Verilog module skeleton for a partitioned network.
 *
 *  Writes the module header, PI/PO declarations, internal wire declarations,
 *  and PO assignments. Submodule instantiations are appended later by
 *  call_submodule(). The caller must write "endmodule\n" to close the file.
 *
 *  Port naming matches mockturtle::write_verilog: PIs are x0,x1,...
 *  and POs are y0,y1,...
 */
template<class Ntk>
void write_toplevel_verilog(
    Ntk& ntk,
    oracle::partition_manager<Ntk>& pm,
    std::string const& filename,
    mockturtle::node_map<std::string, Ntk>& node_names,
    mockturtle::node_map<std::string, Ntk>& input_names,
    std::string const& modulename = "toplevel")
{
    int num_parts = pm.get_part_num();
    std::ofstream os(filename.c_str(), std::ofstream::out);

    // Build PI name list (x0, x1, ...) matching mockturtle::write_verilog convention
    uint32_t num_pis = ntk.num_pis() - ntk.num_latches();
    uint32_t num_pos = ntk.num_pos() - ntk.num_latches();

    std::vector<std::string> pi_names, po_names;
    for (uint32_t i = 0; i < num_pis; ++i)
        pi_names.push_back(fmt::format("x{}", i));
    for (uint32_t i = 0; i < num_pos; ++i)
        po_names.push_back(fmt::format("y{}", i));

    // Build port list string
    std::string ports;
    for (uint32_t i = 0; i < num_pis; ++i) {
        if (i > 0) ports += ", ";
        ports += pi_names[i];
    }
    for (uint32_t i = 0; i < num_pos; ++i) {
        ports += ", ";
        ports += po_names[i];
    }

    os << fmt::format("module {}({});\n", modulename, ports);
    for (uint32_t i = 0; i < num_pis; ++i)
        os << fmt::format("  input {};\n", pi_names[i]);
    for (uint32_t i = 0; i < num_pos; ++i)
        os << fmt::format("  output {};\n", po_names[i]);

    // Initialise node_names and input_names to default
    ntk.foreach_node([&](auto n) {
        input_names[n] = "1'b0";
        node_names[n]  = "1'b0";
    });

    // Assign known names to top-level PIs
    ntk.foreach_pi([&](auto const& n, auto i) {
        if (static_cast<uint32_t>(i) < num_pis)
            input_names[n] = pi_names[i];
    });

    // Collect all partition boundary nodes that need internal wires
    std::set<typename Ntk::node> wire_nodes;
    std::vector<int> vector_output(num_pos, -1);

    for (int p = 0; p < num_parts; ++p) {
        for (auto const& n : pm.get_part_inputs(p))
            if (!ntk.is_pi(n) && !ntk.is_constant(n))
                wire_nodes.insert(n);
        for (auto const& n : pm.get_part_outputs(p))
            if (!ntk.is_pi(n) && !ntk.is_constant(n))
                wire_nodes.insert(n);
    }

    // Declare wires for all PO driver nodes that aren't already PI/constant
    ntk.foreach_po([&](auto const& sig, auto i) {
        if (static_cast<uint32_t>(i) >= num_pos) return;
        auto n = ntk.get_node(sig);
        if (!ntk.is_pi(n) && !ntk.is_constant(n))
            wire_nodes.insert(n);
    });

    // Emit wire declarations and build node_names map
    int wire_count = 0;
    bool first_wire = true;
    std::vector<std::string> wire_list;
    for (auto const& n : wire_nodes) {
        node_names[n] = fmt::format("n{}", wire_count++);
        wire_list.push_back(node_names[n]);
    }
    if (!wire_list.empty()) {
        os << "  wire ";
        for (size_t i = 0; i < wire_list.size(); ++i) {
            if (i > 0) os << ", ";
            os << wire_list[i];
        }
        os << ";\n";
    }
    (void)first_wire; // suppress unused warning

    // Emit PO assignments
    ntk.foreach_po([&](auto const& sig, auto i) {
        if (static_cast<uint32_t>(i) >= num_pos) return;
        auto n = ntk.get_node(sig);
        std::string inv = ntk.is_complemented(sig) ? "~" : "";
        std::string driver;
        if (ntk.is_pi(n))
            driver = input_names[n];
        else if (ntk.is_constant(n))
            driver = ntk.is_complemented(sig) ? "1'b1" : "1'b0";
        else
            driver = node_names[n];
        os << fmt::format("  assign {} = {}{};\n", po_names[i], inv, driver);
    });

    os.flush();
    os.close();
}

/*! \brief Appends a submodule instantiation to the top-level Verilog file.
 *
 *  Port names on the submodule side match mockturtle::write_verilog output:
 *  PIs are x0,x1,... and POs are y0,y1,...
 */
template<class Ntk>
void call_submodule(
    Ntk const& ntk,
    Ntk const& part_ntk,
    std::string const& filename,
    std::string const& modulename,
    int part_num,
    oracle::partition_view<Ntk> const& part,
    mockturtle::node_map<std::string, Ntk>& node_names,
    mockturtle::node_map<std::string, Ntk>& input_names)
{
    std::ofstream os(filename.c_str(), std::ofstream::app);
    os << fmt::format("  {} U{} (", modulename, part_num);

    bool first = true;

    // Connect partition PIs to top-level signals
    part_ntk.foreach_pi([&](auto const& n, auto i) {
        auto idx     = part_ntk.node_to_index(n);
        auto node_id = part.index_to_node(idx);
        if (!first) os << ", ";
        first = false;
        std::string toplevel_sig = ntk.is_pi(node_id)
            ? input_names[node_id]
            : node_names[node_id];
        // Submodule port name matches write_verilog: x0, x1, ...
        os << fmt::format(".x{}({})", i, toplevel_sig);
    });

    // Connect partition POs to top-level wires
    part_ntk.foreach_po([&](auto const& sig, auto i) {
        auto idx     = part_ntk._storage->outputs.at(i).index;
        auto node_id = part.index_to_node(idx);
        if (!first) os << ", ";
        first = false;
        // Submodule port name matches write_verilog: y0, y1, ...
        if (node_names[node_id] == "1'b0")
            os << fmt::format(".y{}()", i);   // unconnected output
        else
            os << fmt::format(".y{}({})", i, node_names[node_id]);
    });

    os << " );\n";
    os.close();
}

} // namespace oracle
