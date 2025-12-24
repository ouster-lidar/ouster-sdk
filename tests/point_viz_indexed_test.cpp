#include <gtest/gtest.h>

#include "indexed.h"
#include "ouster/point_viz.h"

using namespace ouster::sdk::viz;

struct Obj {
    std::string data;
    int update_from_called{0};
    int clear_called{0};
    int draw_called{0};
    int dirty_called{0};

    Obj() = default;
    Obj(std::string d) { data = d; }

    void update_from(Obj& other) {
        *this = other;
        update_from_called++;
    }

    void clear() { clear_called++; }

    void dirty() { dirty_called++; }
};

struct GLObj {
    int draw_called{0};
    GLObj(Obj&) {}
    void draw(const WindowCtx&, const ouster::sdk::viz::impl::CameraData&,
              Obj& obj) {
        draw_called++;
        obj.draw_called++;
    }
};

struct IndexedTest : public Indexed<GLObj, Obj> {
    using Indexed<GLObj, Obj>::back_;
    using Indexed<GLObj, Obj>::front_;
    using Indexed<GLObj, Obj>::back_contains;
    using Indexed<GLObj, Obj>::front_contains;
};

TEST(Indexed, add_remove_1) {
    IndexedTest objs;
    auto obj_a = std::make_shared<Obj>();
    objs.add(obj_a);
    ASSERT_EQ(obj_a->dirty_called, 1);
    ASSERT_TRUE(objs.back_contains(obj_a));
    objs.remove(obj_a);
    ASSERT_FALSE(objs.back_contains(obj_a));

    // adding twice has no effect
    objs.add(obj_a);
    objs.add(obj_a);
    ASSERT_EQ(objs.back_.size(), 1);
}

TEST(Indexed, add_remove_2) {
    IndexedTest objs;
    auto obj_a = std::make_shared<Obj>();
    auto obj_b = std::make_shared<Obj>();
    auto obj_c = std::make_shared<Obj>();
    ASSERT_FALSE(objs.back_contains(obj_a));
    ASSERT_FALSE(objs.back_contains(obj_b));
    objs.add(obj_a);
    objs.add(obj_b);
    objs.add(obj_c);
    ASSERT_TRUE(objs.back_contains(obj_a));
    ASSERT_TRUE(objs.back_contains(obj_b));
    objs.remove(obj_b);
    ASSERT_TRUE(objs.back_contains(obj_a));
    ASSERT_FALSE(objs.back_contains(obj_b));
    ASSERT_TRUE(objs.back_contains(obj_c));
}

TEST(Indexed, add_remove_update_new_and_existing_drawables_1) {
    IndexedTest objs;

    // After add and update_new_and_existing_drawables, the drawable should have
    // a copy of the data for GL to access.
    auto obj_a = std::make_shared<Obj>("data_a");
    auto obj_b = std::make_shared<Obj>("data_b");
    auto obj_c = std::make_shared<Obj>("data_c");
    objs.add(obj_a);
    objs.add(obj_b);
    objs.add(obj_c);
    objs.clean_up_removed_drawables();
    objs.update_new_and_existing_drawables();
    ASSERT_TRUE(objs.front_contains(obj_a));
    ASSERT_TRUE(objs.front_contains(obj_b));
    ASSERT_TRUE(objs.front_contains(obj_c));
    ASSERT_EQ(objs.front_.at(obj_a).state->data, "data_a");
    ASSERT_EQ(objs.front_.at(obj_b).state->data, "data_b");
    ASSERT_EQ(objs.front_.at(obj_c).state->data, "data_c");

    // After remove and update_new_and_existing_drawables the drawable should no
    // longer be in the GL state.
    objs.remove(obj_b);
    objs.clean_up_removed_drawables();
    objs.update_new_and_existing_drawables();
    ASSERT_TRUE(objs.front_contains(obj_a));
    ASSERT_FALSE(objs.front_contains(obj_b));
    ASSERT_TRUE(objs.front_contains(obj_c));

    obj_a->data = "data_a_updated";
    obj_b->data = "data_b_updated";
    obj_c->data = "data_c_updated";

    // After being added and update_new_and_existing_drawables again the state
    // should be restored.
    objs.add(obj_b);
    objs.clean_up_removed_drawables();
    objs.update_new_and_existing_drawables();
    ASSERT_TRUE(objs.front_contains(obj_a));
    ASSERT_TRUE(objs.front_contains(obj_b));
    ASSERT_TRUE(objs.front_contains(obj_c));
    ASSERT_EQ(objs.front_.at(obj_a).state->data, "data_a_updated");
    ASSERT_EQ(objs.front_.at(obj_b).state->data, "data_b_updated");
    ASSERT_EQ(objs.front_.at(obj_c).state->data, "data_c_updated");
}

TEST(Indexed, add_remove_swap_draw_1) {
    // OSDK-291: issue occurred when an object is added, removed,
    // and then another added in its place prior to a call to
    // PointViz::update(). The fix involves detecting if the original instance
    // was replaced at some point.
    IndexedTest objs;
    WindowCtx ctx;
    ouster::sdk::viz::impl::CameraData camera;
    auto obj_a = std::make_shared<Obj>("data_a");
    auto obj_b = std::make_shared<Obj>("data_b");

    // ADD A
    objs.add(obj_a);
    ASSERT_EQ(obj_a->clear_called, 0);
    ASSERT_TRUE(objs.back_contains(obj_a));

    // REMOVE A ADD B
    objs.remove(obj_a);
    ASSERT_FALSE(objs.back_contains(obj_a));
    objs.add(obj_b);
    ASSERT_TRUE(objs.back_contains(obj_b));
    objs.update_new_and_existing_drawables();
    objs.clean_up_removed_drawables();
    ASSERT_FALSE(objs.front_contains(obj_a));
    ASSERT_TRUE(objs.front_contains(obj_b));
    objs.draw(ctx, camera);

    ASSERT_EQ(objs.front_.size(), 1);
    auto& front_item = objs.front_.begin()->second;
    ASSERT_EQ(front_item.state->clear_called, 0);
    ASSERT_EQ(front_item.state->data, "data_b");

    ASSERT_EQ(front_item.state->draw_called, 1);
    ASSERT_EQ(front_item.gl->draw_called, 1);
    ASSERT_EQ(front_item.state->update_from_called, 1);

    // REMOVE B ADD A
    objs.remove(obj_b);
    objs.add(obj_a);
    ASSERT_TRUE(objs.back_contains(obj_a));
    auto& front_item_2 = objs.front_.begin()->second;
    ASSERT_EQ(front_item_2.state->dirty_called, 1);
    objs.update_new_and_existing_drawables();
    objs.clean_up_removed_drawables();

    auto& front_item_3 = objs.front_.begin()->second;
    ASSERT_EQ(front_item_3.state->dirty_called, 2);
    objs.draw(ctx, camera);

    ASSERT_EQ(objs.front_.size(), 1);
    auto& front_item_4 = objs.front_.begin()->second;
    ASSERT_EQ(front_item_4.state->draw_called, 1);
    ASSERT_EQ(front_item_4.state->data, "data_a");

    // The GL wrapper for this drawable was called exactly once (because it is
    // new.)
    ASSERT_EQ(front_item_4.state->draw_called, 1);
    ASSERT_EQ(front_item_4.gl->draw_called, 1);
    ASSERT_EQ(front_item_4.state->update_from_called, 1);
}

TEST(Indexed, removeUpdateRaceCondition) {
    /*
    A test for a fix of the following race condition:

    User calls PointViz::add(foo)

    User calls PointViz::update().
    User calls PointViz::remove(foo)

    Draw thread calls PointViz::clean_up_removed_drawables() followed by
    PointViz::draw() shortly thereafter causing the widget to disappear prior to
    the next call to PointViz::update().

    In this scenario, objects disappear prior to when the user intends them to,
    resulting in flickering drawables. The fix is now that
    Indexed::update_new_and_existing_drawables() (called only when update() is
    called) adds removed drawables to a "death queue", which is cleared at the
    beginning of a subsequent call to draw().
    */
    static int destructor_called = false;
    struct Tmp : public Obj {
        ~Tmp() {
            // will indicate that the unique ptr for the "front" was deleted
            destructor_called = true;
        }
    };
    struct IndexedTestTmp : public Indexed<GLObj, Tmp> {
        using Indexed<GLObj, Tmp>::back_;
        using Indexed<GLObj, Tmp>::front_;
        using Indexed<GLObj, Tmp>::back_contains;
        using Indexed<GLObj, Tmp>::front_contains;
    };
    // Drawable state should not be marked for deletion until update is called.
    IndexedTestTmp objs;
    ouster::sdk::viz::impl::CameraData camera;
    auto obj_a = std::make_shared<Tmp>();

    // ADD A
    objs.add(obj_a);
    ASSERT_EQ(obj_a->clear_called, 0);
    ASSERT_TRUE(objs.back_contains(obj_a));
    ASSERT_FALSE(objs.front_contains(obj_a));

    objs.update_new_and_existing_drawables();

    // REMOVE A
    objs.remove(obj_a);

    // A is still in front even after removed drawables have been cleaned up
    objs.clean_up_removed_drawables();
    ASSERT_FALSE(destructor_called);
    ASSERT_TRUE(objs.front_contains(obj_a));
    ASSERT_FALSE(objs.back_contains(obj_a));

    // After update, A is no longer in front or back
    objs.update_new_and_existing_drawables();
    ASSERT_FALSE(destructor_called);
    ASSERT_FALSE(objs.front_contains(obj_a));
    ASSERT_FALSE(objs.back_contains(obj_a));

    // Our fron copy of A is finally cleaned up after a subsequent call to
    // clean_up_removed_drawables
    ASSERT_FALSE(destructor_called);
    objs.clean_up_removed_drawables();
    ASSERT_TRUE(destructor_called);
}
