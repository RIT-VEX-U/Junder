#include <type_traits>
#include <utility>

template <typename System, typename IDType, typename Message, int32_t delay_ms,
          bool do_log = false>
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
        virtual State *respond(System &s, Message m) = 0;
        // Identify
        virtual IDType id() const = 0;

        // virtual destructor cuz c++
        virtual ~State() {}
    };
    using callback_data = std::pair<State *, StateMachine *>;
    StateMachine(State *initial)
        : runner(thread_runner, new callback_data{initial, this}) {}
    static int thread_runner(void *vptr) {
        callback_data *ptr = static_cast<callback_data *>(vptr);
        State *cur_state = ptr->first;

        StateMachine &sys = *ptr->second;
        System &derived = *static_cast<System *>(&sys);

        cur_state->entry(derived);

        auto respond_to_message = [&](Message msg) {
            if (do_log) {
                printf("responding to msg: %s\n", to_string(msg).c_str());
                fflush(stdout);
            }

            State *next_state = cur_state->respond(derived, msg);

            if (cur_state != next_state) {
                sys.mut.lock();

                cur_state->exit(derived);
                next_state->entry(derived);

                delete cur_state;

                cur_state = next_state;
                sys.cur_type = cur_state->id();

                sys.mut.unlock();
            }
        };

        while (true) {
            if (do_log) {
                std::string str = to_string(cur_state->id());
                printf("state: %s\n", str.c_str());
            }

            // Internal Message passed
            MaybeMessage internal_msg = cur_state->work(derived);

            if (internal_msg.has_message()) {
                respond_to_message(internal_msg.message());
            }

            // External Message passed
            sys.mut.lock();
            MaybeMessage incoming = sys.incoming_msg;
            sys.incoming_msg = {};
            sys.mut.unlock();

            if (incoming.has_message()) {
                respond_to_message(incoming.message());
            }
            vexDelay(delay_ms);
        }
    }
    IDType current_state() const {
        mut.lock();
        auto t = cur_type;
        mut.unlock();
        return t;
    }
    void SendMessage(Message msg) {
        mut.lock();
        incoming_msg = msg;
        mut.unlock();
    }

  private:
    vex::task runner;
    mutable vex::mutex mut;
    MaybeMessage incoming_msg;
    IDType cur_type;
};
