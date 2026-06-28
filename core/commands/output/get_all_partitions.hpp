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
#include <alice/alice.hpp>

#include <mockturtle/mockturtle.hpp>
#include <nlohmann/json.hpp>

#include <stdio.h>
#include <fstream>
#include <unordered_map>

#include <sys/stat.h>
#include <stdlib.h>
#include <filesystem>

#include "algorithms/output/verilog_utilities.hpp"


namespace alice
{

// ---------------------------------------------------------------------------
// write_window_verilog
//
// Writes a window_view directly as a structural Verilog module.
// Port naming: inputs  → x0, x1, ...
//              outputs → y0, y1, ...
// Internal gate wires are named n<original_node_index> (unique globally).
// No node_resynthesis is used; this preserves the original node indices so
// the connectivity map built in execute() stays accurate.
// ---------------------------------------------------------------------------
template<class WindowView>
static void write_window_verilog(WindowView const& wv,
                                 std::string const& filename,
                                 std::string const& modname)
{
    using node   = typename WindowView::node;
    using signal = typename WindowView::signal;

    // Map from original node index → local name used in this file
    std::unordered_map<uint32_t, std::string> name_of;

    // Assign x0, x1, ... to PI nodes (window inputs)
    int pi_count = 0;
    wv.foreach_pi([&](node n, int) {
        name_of[wv.node_to_index(n)] = "x" + std::to_string(pi_count++);
    });

    // Assign n<idx> to internal gate nodes
    wv.foreach_node([&](node n) {
        if (!wv.is_constant(n) && !wv.is_pi(n))
            name_of[wv.node_to_index(n)] = "n" + std::to_string(wv.node_to_index(n));
    });

    // Helper: resolve a signal to a Verilog expression
    auto sig_expr = [&](signal s) -> std::string {
        node n = wv.get_node(s);
        if (wv.is_constant(n))
            return wv.is_complemented(s) ? "1'b1" : "1'b0";
        auto it = name_of.find(wv.node_to_index(n));
        std::string nm = (it != name_of.end()) ? it->second
                         : ("n" + std::to_string(wv.node_to_index(n)));
        return wv.is_complemented(s) ? ("~" + nm) : nm;
    };

    // Collect port lists
    std::vector<std::string> in_ports, out_ports;
    wv.foreach_pi([&](node, int i) {
        in_ports.push_back("x" + std::to_string(i));
    });
    for (int i = 0; i < (int)wv.num_pos(); ++i)
        out_ports.push_back("y" + std::to_string(i));

    std::ofstream os(filename);

    // Module header
    os << "module " << modname << "(";
    bool first = true;
    for (auto& p : in_ports)  { if (!first) os << " , "; os << p; first = false; }
    for (auto& p : out_ports) { if (!first) os << " , "; os << p; first = false; }
    os << " );\n";

    if (!in_ports.empty()) {
        os << "  input";
        first = true;
        for (auto& p : in_ports) { os << (first ? " " : " , ") << p; first = false; }
        os << " ;\n";
    }
    if (!out_ports.empty()) {
        os << "  output";
        first = true;
        for (auto& p : out_ports) { os << (first ? " " : " , ") << p; first = false; }
        os << " ;\n";
    }

    // Wire declarations for internal gates
    bool any_gate = false;
    wv.foreach_node([&](node n) {
        if (wv.is_constant(n) || wv.is_pi(n)) return;
        if (!any_gate) { os << "  wire"; any_gate = true; first = true; }
        os << (first ? " " : " , ") << name_of[wv.node_to_index(n)];
        first = false;
    });
    if (any_gate) os << " ;\n";

    // Assign statements (AIG: 2-input AND)
    wv.foreach_node([&](node n) {
        if (wv.is_constant(n) || wv.is_pi(n)) return;
        os << "  assign " << name_of[wv.node_to_index(n)] << " = ";
        bool fst = true;
        wv.foreach_fanin(n, [&](signal s) {
            if (!fst) os << " & ";
            os << sig_expr(s);
            fst = false;
        });
        os << " ;\n";
    });

    // PO assignments
    wv.foreach_po([&](signal s, int i) {
        os << "  assign " << out_ports[i] << " = " << sig_expr(s) << " ;\n";
    });

    os << "endmodule\n";
}


class get_all_partitions_command : public alice::command
{

public:
    explicit get_all_partitions_command(const environment::ptr &env)
        : command(env, "Exports every partition to Verilog files")
    {
        opts.add_option("--directory,directory", dir,
                        "Directory to write Verilog files to")->required();
        add_flag("--mig,-m",
                 "Write out all of the partitions of the sored MIG network");
    }

protected:
    void execute()
    {
        mockturtle::mig_npn_resynthesis resyn_mig;
        if (is_set("mig")) {
            if (!store<mig_ntk>().empty()) {
                auto ntk = *store<mig_ntk>().current();
                env->out() << "\n";
                if (!store<part_man_mig_ntk>().empty()) {
                    auto partitions = *store<part_man_mig_ntk>().current();
                    for (int i = 0; i < partitions.get_part_num(); i++) {
                        auto part_outputs = partitions.get_part_outputs(i);
                        env->out() << "Partition " << i << ":\n";
                        env->out() << "Number of Logic Cones = "
                                   << part_outputs.size() << "\n";
                        mkdir(dir.c_str(), 0777);

                        oracle::partition_view<mig_names> part =
                            partitions.create_part(ntk, i);
                        auto part_ntk =
                            mockturtle::node_resynthesis<mockturtle::mig_network>(
                                part, resyn_mig);

                        std::string filename = dir + "/" + ntk.get_network_name()
                                               + "_" + std::to_string(i) + ".v";
                        mockturtle::write_verilog(part_ntk, filename);
                        env->out() << "\n";
                    }
                } else {
                    env->err() << "Partitions have not been mapped\n";
                }
            } else {
                env->err() << "There is no MIG network stored\n";
            }
            return;
        }

        // ---- AIG path ----
        if (store<aig_ntk>().empty()) {
            env->err() << "There is no AIG network stored\n";
            return;
        }
        if (store<part_man_jr_aig_ntk>().empty()) {
            env->err() << "Partitions have not been mapped\n";
            return;
        }

        auto &pm  = *store<part_man_jr_aig_ntk>().current();
        auto  ntk = pm.get_network();

        std::string network_name = ntk.get_network_name().empty()
                                   ? "top" : ntk.get_network_name();
        std::string toplevel_module =
            std::filesystem::path(network_name).filename().string();

        mkdir(dir.c_str(), 0777);
        env->out() << "\n";

        // ---- Write each partition as a structural Verilog module ----
        for (int i = 0; i < pm.count(); i++) {
            auto part = pm.partition(i);

            if (part.num_pos() == 0)
                continue;

            env->out() << "Partition " << i << ":\n";

            std::string modname = std::filesystem::path(
                                      toplevel_module + "_" + std::to_string(i)
                                  ).filename().string();
            std::string filename = dir + "/" + modname + ".v";

            write_window_verilog(part, filename, modname);

            env->out() << "\n";
        }

        // ---- Write connectivity.json ----
        // Captures all information the Python top-level generator needs:
        // per-partition port-to-original-node-index mapping, network PI/PO/RI/RO lists.
        nlohmann::json conn;
        conn["network_name"]   = toplevel_module;
        conn["num_pis"]        = ntk.num_pis();
        conn["num_pos"]        = ntk.num_pos();
        conn["num_registers"]  = ntk.num_registers();

        // Primary inputs
        {
            auto arr = nlohmann::json::array();
            ntk.foreach_pi([&](auto n, int) {
                arr.push_back(ntk.node_to_index(n));
            });
            conn["pi_nodes"] = arr;
        }

        // Register outputs (current state, i.e. latch Q)
        {
            auto arr = nlohmann::json::array();
            ntk.foreach_ro([&](auto n, int) {
                arr.push_back(ntk.node_to_index(n));
            });
            conn["ro_nodes"] = arr;
        }

        // Register inputs (next state, i.e. latch D)
        {
            auto narr = nlohmann::json::array();
            auto carr = nlohmann::json::array();
            ntk.foreach_ri([&](auto s, int) {
                narr.push_back(ntk.node_to_index(ntk.get_node(s)));
                carr.push_back(ntk.is_complemented(s));
            });
            conn["ri_nodes"]       = narr;
            conn["ri_complements"] = carr;
        }

        // Primary outputs
        {
            auto narr = nlohmann::json::array();
            auto carr = nlohmann::json::array();
            ntk.foreach_po([&](auto s, int) {
                narr.push_back(ntk.node_to_index(ntk.get_node(s)));
                carr.push_back(ntk.is_complemented(s));
            });
            conn["po_nodes"]       = narr;
            conn["po_complements"] = carr;
        }

        // Per-partition port mapping:
        // inputs[i]  = original node index for xi port
        // outputs[i] = original node index for yi port
        {
            auto parts_arr = nlohmann::json::array();
            for (int i = 0; i < pm.count(); i++) {
                auto [inp, out] = pm.partition_io(i);
                nlohmann::json pj;
                pj["id"] = i;
                {
                    auto arr = nlohmann::json::array();
                    for (auto n : inp) arr.push_back(ntk.node_to_index(n));
                    pj["inputs"] = arr;
                }
                {
                    auto arr = nlohmann::json::array();
                    for (auto s : out)
                        arr.push_back(ntk.node_to_index(ntk.get_node(s)));
                    pj["outputs"] = arr;
                }
                parts_arr.push_back(pj);
            }
            conn["partitions"] = parts_arr;
        }

        std::string conn_path = dir + "/connectivity.json";
        std::ofstream conn_os(conn_path);
        conn_os << conn.dump(2) << "\n";
        conn_os.close();
        env->out() << "Wrote " << conn_path << "\n";
    }

private:
    std::string dir{};
};

ALICE_ADD_COMMAND(get_all_partitions, "Output");
}
