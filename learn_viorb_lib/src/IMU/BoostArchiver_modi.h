/*
 * map save/load extension for LearnVIORB
 * This header contains boost headers needed by serialization
 *
 * object to save:
 *   - KeyFrame
 *   - KeyFrameDatabase
 *   - Map
 *   - MapPoint
 */
#ifndef BOOST_ARCHIVER_H
#define BOOST_ARCHIVER_H
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
// set serialization needed by KeyFrame::mspChildrens ...
#include <boost/serialization/map.hpp>
// map serialization needed by KeyFrame::mConnectedKeyFrameWeights ...
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>


namespace boost{
    namespace serialization {


    }
}
// TODO: boost::iostream zlib compressed binary format
#endif // BOOST_ARCHIVER_H
