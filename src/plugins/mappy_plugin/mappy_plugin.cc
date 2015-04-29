#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "plugin.hh"
#include "utils/function_property_accessor.hh"

using namespace std;

namespace Tempus {


class CostCalculator
{
public:
    double operator() ( const Multimodal::Graph& graph, const Multimodal::Edge& e )
    {
        
        return graph.road()[e.road_edge()].length();
    }
};

class MappyPlugin : public Plugin {
public:

    static const OptionDescriptionList option_descriptions() {
        return OptionDescriptionList();
    }

    static const PluginCapabilities plugin_capabilities() {
        PluginCapabilities params;
        params.optimization_criteria().push_back( CostDistance );
        params.set_intermediate_steps( false );
        params.set_depart_after( true );
        params.set_arrive_before( false );
        return params;
    }

    MappyPlugin( const std::string& nname, const std::string db_options ) : Plugin( nname, db_options ) {
    }

    virtual ~MappyPlugin() {
    }

private:

public:

    static void post_build() {
    }

    virtual void pre_process( Request& request ) {
        request_ = request;
        result_.clear();
        iterations_ = 0;
    }


    // the exception to throw to short cut dijkstra
    struct PathFound {};

    // current destination
    Multimodal::Vertex destination_;

    int iterations_;

    struct path_found_exception {};

    virtual void vertex_accessor( const Multimodal::Vertex& v, int access_type ) {
        if ( access_type == Plugin::ExamineAccess ) {
            iterations_++;
            if ( v == destination_ ) {
                throw path_found_exception();
            }

            //            std::cout << v << std::endl;
        }
    }

    ///
    /// The main process
    virtual void process() {
        size_t n = num_vertices( graph_ );

        std::vector<boost::default_color_type> color_map( n );
        std::vector<Multimodal::Vertex> pred_map( n );
        // distance map (in number of nodes)
        std::vector<double> node_distance_map( n );

        Multimodal::VertexIndexProperty vertex_index = get( boost::vertex_index, graph_ );

        Tempus::PluginGraphVisitor vis( this );

        CostCalculator cost_calculator;
        Tempus::FunctionPropertyAccessor<Multimodal::Graph,
                                         boost::edge_property_tag,
                                         double,
                                         CostCalculator> cost_map( graph_, cost_calculator );

        Multimodal::Vertex origin( &graph_.road(), request_.origin() );
        std::cout << "origin: " << origin << std::endl;
        destination_ = Multimodal::Vertex( &graph_.road(), request_.destination() );
        std::cout << "destination: " << destination_ << std::endl;

        boost::iterator_property_map<std::vector<Multimodal::Vertex>::iterator,
                                     Multimodal::VertexIndexProperty,
                                     Multimodal::Vertex,
                                     Multimodal::Vertex&> pred_pmap = boost::make_iterator_property_map( pred_map.begin(), vertex_index );

        bool path_found = false;
        try {
            boost::dijkstra_shortest_paths( graph_,
                                            origin,
                                            pred_pmap,
                                            boost::make_iterator_property_map( node_distance_map.begin(), vertex_index ),
                                            cost_map,
                                            vertex_index,
                                            std::less<double>(),
                                            boost::closed_plus<double>(),
                                            std::numeric_limits<double>::max(),
                                            0.0,
                                            vis,
                                            boost::make_iterator_property_map( color_map.begin(), vertex_index )
                                            );
        } catch ( path_found_exception& ) {
            //
            path_found = true;
        }

        if ( !path_found ) {
            std::cout << "path not found!" << std::endl;
            return;
        }

        // reorder the path, could have been better included ...
        std::list<Multimodal::Vertex> path;
        Multimodal::Vertex current = destination_;

        while ( current != origin ) {
            path.push_front( current );
            Multimodal::Vertex v = get( pred_pmap, current );
            std::cout << "previous of " << current << " is " << v << std::endl;
            current = v;
            //            current = pred_map[ current ];
        }

        result_.push_back(Roadmap());
        Roadmap& roadmap = result_.back();
        roadmap.set_starting_date_time( request_.steps()[1].constraint().date_time() );

        std::list<Multimodal::Vertex>::iterator prev = path.begin();
        std::list<Multimodal::Vertex>::iterator it = prev;
        it++;

        for ( ; it != path.end(); ++it, ++prev) {
            Multimodal::Vertex v = *it;
            Multimodal::Vertex previous = *prev;

            Road::Edge e;
            bool found = false;
            //            std::cout << previous.road_vertex() << " - " << v.road_vertex() << std::endl;
            boost::tie( e, found ) = boost::edge( previous.road_vertex(), v.road_vertex(), graph_.road() );
            if (!found) {
                std::cout << "errrroooooorrrr" << std::endl;
                continue;
            }

            Roadmap::RoadStep* step = new Roadmap::RoadStep();
            step->set_cost( CostDistance, graph_.road()[e].length() );
            step->set_road_edge( e );
            step->set_transport_mode( 1 );
            roadmap.add_step( std::auto_ptr<Roadmap::Step>(step) );
        }

        metrics_["iterations"] = iterations_;
    }

    void cleanup() {
        // nothing special to clean up
    }

};
}
DECLARE_TEMPUS_PLUGIN( "mappy_plugin", Tempus::MappyPlugin )
