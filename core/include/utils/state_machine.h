#include <type_traits>

template <typename System, typename IDType, typename Message,
          bool do_log = true>
class StateMachine {
    static_assert(std::is_enum<Message>::value, "message should be an enum");

  public:
    struct MaybeMessage {
        MaybeMessage() : exists(false) {}
        MaybeMessage(Message msg) : exists(true), thing(msg) {}
        bool has_message() { return exists; }
        Message message() { return thing; }
        bool exists;
        Message thing;
    };
    struct State {
        // run once when we enter the state
        virtual void entry(System &) {}
        // run continously while in the state
        virtual MaybeMessage work(System &) { return {}; }
        // run once when we exit the state
        virtual void exit(System &) {}
        // respond to a message when one comes in
        virtual State *respond(Message m) = 0;
        // Identify
        virtual IDType id() const = 0;

        // virtual destructor cuz c++
        virtual ~State() {}
    };
    void run(State *initial) {
        State *cur_state = initial;
        System &sys = *static_cast<System *>(this);
        while (true) {
            while (true) {
                if (do_log) {
                    std::string str = to_string(cur_state->id());
                    printf("state: %s", str.c_str());
                }

                MaybeMessage gotten = cur_state->work(sys);
                if (gotten.has_message()) {
                    State *next_state = cur_state->respond(gotten.message());
                    if (cur_state != next_state) {
                        cur_state->exit(sys);
                        next_state->entry(sys);
                        delete cur_state;
                        cur_state = next_state;
                    }
                }
            }
        }
    }
    IDType current_state() {}
};
