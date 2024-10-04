// Copyright 2013-2023 Daniel Parker
// Distributed under the Boost license, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

// See https://github.com/danielaparker/jsoncons for latest version

#ifndef JSONCONS_JSONPATH_JSONPATH_SELECTOR_HPP
#define JSONCONS_JSONPATH_JSONPATH_SELECTOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <type_traits> // std::is_const
#include <limits> // std::numeric_limits
#include <utility> // std::move
#include <regex>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/jsonpath_error.hpp>
#include <jsoncons_ext/jsonpath/expression.hpp>

namespace jsoncons { 
namespace jsonpath {
namespace detail {

    struct slice
    {
        jsoncons::optional<int64_t> start_;
        jsoncons::optional<int64_t> stop_;
        int64_t step_;

        slice()
            : start_(), stop_(), step_(1)
        {
        }

        slice(const jsoncons::optional<int64_t>& start, const jsoncons::optional<int64_t>& end, int64_t step) 
            : start_(start), stop_(end), step_(step)
        {
        }

        slice(const slice& other)
            : start_(other.start_), stop_(other.stop_), step_(other.step_)
        {
        }

        slice& operator=(const slice& rhs) 
        {
            if (this != &rhs)
            {
                if (rhs.start_)
                {
                    start_ = rhs.start_;
                }
                else
                {
                    start_.reset();
                }
                if (rhs.stop_)
                {
                    stop_ = rhs.stop_;
                }
                else
                {
                    stop_.reset();
                }
                step_ = rhs.step_;
            }
            return *this;
        }

        int64_t get_start(std::size_t size) const
        {
            if (start_)
            {
                auto len = *start_ >= 0 ? *start_ : (static_cast<int64_t>(size) + *start_);
                return len <= static_cast<int64_t>(size) ? len : static_cast<int64_t>(size);
            }
            else
            {
                if (step_ >= 0)
                {
                    return 0;
                }
                else 
                {
                    return static_cast<int64_t>(size);
                }
            }
        }

        int64_t get_stop(std::size_t size) const
        {
            if (stop_)
            {
                auto len = *stop_ >= 0 ? *stop_ : (static_cast<int64_t>(size) + *stop_);
                return len <= static_cast<int64_t>(size) ? len : static_cast<int64_t>(size);
            }
            else
            {
                return step_ >= 0 ? static_cast<int64_t>(size) : -1;
            }
        }

        int64_t step() const
        {
            return step_; // Allow negative
        }
    };

    template <class Json,class JsonReference>
    class json_array_receiver : public node_receiver<Json,JsonReference>
    {
    public:
        using reference = JsonReference;
        using char_type = typename Json::char_type;
        using string_type = typename Json::string_type;
        using path_node_type = basic_path_node<typename Json::char_type>;

        Json* val;

        json_array_receiver(Json* ptr)
            : val(ptr)
        {
        }

        void add(const path_node_type&, reference value) override
        {
            val->emplace_back(value);
        }
    };

    template <class Json,class JsonReference>
    struct path_generator
    {
        using char_type = typename Json::char_type;
        using string_view_type = typename Json::string_view_type;
        using string_type = typename Json::string_type;
        using path_node_type = basic_path_node<typename Json::char_type>;

        static const path_node_type& generate(dynamic_resources<Json,JsonReference>& resources,
            const path_node_type& last, 
            std::size_t index, 
            result_options options) 
        {
            const result_options require_path = result_options::path | result_options::nodups | result_options::sort;
            if ((options & require_path) != result_options())
            {
                return *resources.create_path_node(&last, index);
            }
            else
            {
                return last;
            }
        }

        static const path_node_type& generate(dynamic_resources<Json,JsonReference>& resources,
            const path_node_type& last, 
            const string_view_type& identifier, 
            result_options options) 
        {
            const result_options require_path = result_options::path | result_options::nodups | result_options::sort;
            if ((options & require_path) != result_options())
            {
                return *resources.create_path_node(&last, identifier);
            }
            else
            {
                return last;
            }
        }
    };

    template <class Json,class JsonReference>
    class base_selector : public jsonpath_selector<Json,JsonReference>
    {
        using supertype = jsonpath_selector<Json,JsonReference>;

        supertype* tail_;
    public:
        using char_type = typename Json::char_type;
        using string_type = typename Json::string_type;
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using node_receiver_type = typename supertype::node_receiver_type;
        using selector_type = typename supertype::selector_type;

        base_selector()
            : supertype(true, 11), tail_(nullptr)
        {
        }

        base_selector(bool is_path, std::size_t precedence_level)
            : supertype(is_path, precedence_level), tail_(nullptr)
        {
        }

        void append_selector(selector_type* expr) override
        {
            if (!tail_)
            {
                tail_ = expr;
            }
            else
            {
                tail_->append_selector(expr);
            }
        }

        void tail_select(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current,
                           node_receiver_type& receiver,
                           result_options options) const
        {
            if (!tail_)
            {
                receiver.add(last, current);
            }
            else
            {
                tail_->select(resources, root, last, current, receiver, options);
            }
        }

        reference evaluate_tail(dynamic_resources<Json,JsonReference>& resources,
                                reference root,
                                const path_node_type& last, 
                                reference current, 
                                result_options options,
                                std::error_code& ec) const
        {
            if (!tail_)
            {
                return current;
            }
            else
            {
                return tail_->evaluate(resources, root, last, current, options, ec);
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            if (tail_)
            {
                s.append(tail_->to_string(level));
            }
            return s;
        }
    };

    template <class Json,class JsonReference>
    class identifier_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;
        using path_generator_type = path_generator<Json,JsonReference>;
    public:
        using char_type = typename Json::char_type;
        using string_type = typename Json::string_type;
        using string_view_type = typename Json::string_view_type;
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using node_receiver_type = typename supertype::node_receiver_type;
    private:
        string_type identifier_;
    public:

        identifier_selector(const string_type& identifier)
            : base_selector<Json,JsonReference>(), identifier_(identifier)
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_object())
            {
                auto it = current.find(identifier_);
                if (it != current.object_range().end())
                {
                    this->tail_select(resources, root, 
                                        path_generator_type::generate(resources, last, identifier_, options),
                                        it->value(), receiver, options);
                }
            }
            else if (current.is_array())
            {
                int64_t n{0};
                auto r = jsoncons::detail::to_integer_decimal(identifier_.data(), identifier_.size(), n);
                if (r)
                {
                    std::size_t index = (n >= 0) ? static_cast<std::size_t>(n) : static_cast<std::size_t>(static_cast<int64_t>(current.size()) + n);
                    if (index < current.size())
                    {
                        this->tail_select(resources, root, 
                                            path_generator_type::generate(resources, last, index, options),
                                            current[index], receiver, options);
                    }
                }
                else if (identifier_ == resources.length_label() && current.size() >= 0)
                {
                    pointer ptr = resources.create_json(current.size(), semantic_tag::none, resources.get_allocator());
                    this->tail_select(resources, root, 
                                        path_generator_type::generate(resources, last, identifier_, options), 
                                        *ptr, 
                                        receiver, options);
                }
            }
            else if (current.is_string() && identifier_ == resources.length_label())
            {
                string_view_type sv = current.as_string_view();
                std::size_t count = unicode_traits::count_codepoints(sv.data(), sv.size());
                pointer ptr = resources.create_json(count, semantic_tag::none, resources.get_allocator());
                this->tail_select(resources, root, 
                                    path_generator_type::generate(resources, last, identifier_, options), 
                                    *ptr, receiver, options);
            }
            //std::cout << "end identifier_selector\n";
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code& ec) const override
        {
            if (current.is_object())
            {
                auto it = current.find(identifier_);
                if (it != current.object_range().end())
                {
                    return this->evaluate_tail(resources, root, 
                                               path_generator_type::generate(resources, last, identifier_, options),
                                              it->value(), options, ec);
                }
                else
                {
                    return resources.null_value();
                }
            }
            else if (current.is_array())
            {
                int64_t n{0};
                auto r = jsoncons::detail::to_integer_decimal(identifier_.data(), identifier_.size(), n);
                if (r)
                {
                    std::size_t index = (n >= 0) ? static_cast<std::size_t>(n) : static_cast<std::size_t>(static_cast<int64_t>(current.size()) + n);
                    if (index < current.size())
                    {
                        return this->evaluate_tail(resources, root, 
                                                   path_generator_type::generate(resources, last, index, options),
                                                   current[index], options, ec);
                    }
                    else
                    {
                        return resources.null_value();
                    }
                }
                else if (identifier_ == resources.length_label() && current.size() > 0)
                {
                    pointer ptr = resources.create_json(current.size(), semantic_tag::none, resources.get_allocator());
                    return this->evaluate_tail(resources, root, 
                                               path_generator_type::generate(resources, last, identifier_, options), 
                                               *ptr, 
                                               options, ec);
                }
                else
                {
                    return resources.null_value();
                }
            }
            else if (current.is_string() && identifier_ == resources.length_label())
            {
                string_view_type sv = current.as_string_view();
                std::size_t count = unicode_traits::count_codepoints(sv.data(), sv.size());
                pointer ptr = resources.create_json(count, semantic_tag::none, resources.get_allocator());
                return this->evaluate_tail(resources, root, 
                                           path_generator_type::generate(resources, last, identifier_, options), 
                                           *ptr, options, ec);
            }
            else
            {
                return resources.null_value();
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("identifier selector ");
            unicode_traits::convert(identifier_.data(),identifier_.size(),s);
            s.append(base_selector<Json,JsonReference>::to_string(level+1));
            //s.append("\n");

            return s;
        }
    };

    template <class Json,class JsonReference>
    class root_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;
        using path_generator_type = path_generator<Json,JsonReference>;

        std::size_t id_;
    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using node_receiver_type = typename supertype::node_receiver_type;

        root_selector(std::size_t id)
            : base_selector<Json,JsonReference>(), id_(id)
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
                this->tail_select(resources, root, last, root, receiver, options);
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference, 
                           result_options options,
                           std::error_code& ec) const override
        {
            if (resources.is_cached(id_))
            {
                return resources.retrieve_from_cache(id_);
            }
            else
            {
                auto& ref = this->evaluate_tail(resources, root, last, root, options, ec);
                if (!ec)
                {
                    resources.add_to_cache(id_, ref);
                }

                return ref;
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("root_selector ");
            s.append(base_selector<Json,JsonReference>::to_string(level+1));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class current_node_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        current_node_selector()
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            this->tail_select(resources,  
                                root, last, current, receiver, options);
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code& ec) const override
        {
            //std::cout << "current_node_selector: " << current << "\n";
            return this->evaluate_tail(resources,  
                                root, last, current, options, ec);
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("current_node_selector");
            s.append(base_selector<Json,JsonReference>::to_string(level+1));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class parent_node_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;
        using allocator_type = typename Json::allocator_type;

        int ancestor_depth_;

    public:
        using char_type = typename Json::char_type;
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        parent_node_selector(int ancestor_depth)
        {
            ancestor_depth_ = ancestor_depth;
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            const path_node_type* ancestor = std::addressof(last);
            int index = 0;
            while (ancestor != nullptr && index < ancestor_depth_)
            {
                ancestor = ancestor->parent();
                ++index;
            }

            if (ancestor != nullptr)
            {
                pointer ptr = jsoncons::jsonpath::select(root,*ancestor);
                if (ptr != nullptr)
                {
                    this->tail_select(resources, root, *ancestor, *ptr, receiver, options);
                }
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference, 
                           result_options options,
                           std::error_code& ec) const override
        {
            const path_node_type* ancestor = std::addressof(last);
            int index = 0;
            while (ancestor != nullptr && index < ancestor_depth_)
            {
                ancestor = ancestor->parent();
                ++index;
            }

            if (ancestor != nullptr)
            {
                pointer ptr = jsoncons::jsonpath::select(root, *ancestor);
                if (ptr != nullptr)
                {
                    return this->evaluate_tail(resources, root, *ancestor, *ptr, options, ec);
                }
                else
                {
                    return resources.null_value();
                }
            }
            else
            {
                return resources.null_value();
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("parent_node_selector");
            s.append(base_selector<Json,JsonReference>::to_string(level+1));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class index_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

        int64_t index_;
    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        index_selector(int64_t index)
            : base_selector<Json,JsonReference>(), index_(index)
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_array())
            {
                int64_t slen = static_cast<int64_t>(current.size());
                if (index_ >= 0 && index_ < slen)
                {
                    std::size_t i = static_cast<std::size_t>(index_);
                    this->tail_select(resources, root, 
                                        path_generator_type::generate(resources, last, i, options), 
                                        current.at(i), receiver, options);
                }
                else 
                {
                    int64_t index = slen + index_;
                    if (index >= 0 && index < slen)
                    {
                        std::size_t i = static_cast<std::size_t>(index);
                        this->tail_select(resources, root, 
                                            path_generator_type::generate(resources, last, i, options), 
                                            current.at(i), receiver, options);
                    }
                }
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code& ec) const override
        {
            if (current.is_array())
            {
                int64_t slen = static_cast<int64_t>(current.size());
                if (index_ >= 0 && index_ < slen)
                {
                    std::size_t i = static_cast<std::size_t>(index_);
                    return this->evaluate_tail(resources, root, 
                                        path_generator_type::generate(resources, last, i, options), 
                                        current.at(i), options, ec);
                }
                else 
                {
                    int64_t index = slen + index_;
                    if (index >= 0 && index < slen)
                    {
                        std::size_t i = static_cast<std::size_t>(index);
                        return this->evaluate_tail(resources, root, 
                                            path_generator_type::generate(resources, last, i, options), 
                                            current.at(i), options, ec);
                    }
                    else
                    {
                        return resources.null_value();
                    }
                }
            }
            else
            {
                return resources.null_value();
            }
        }
    };

    template <class Json,class JsonReference>
    class wildcard_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        wildcard_selector()
            : base_selector<Json,JsonReference>()
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_array())
            {
                for (std::size_t i = 0; i < current.size(); ++i)
                {
                    this->tail_select(resources, root, 
                                        path_generator_type::generate(resources, last, i, options), current[i], 
                                        receiver, options);
                }
            }
            else if (current.is_object())
            {
                for (auto& member : current.object_range())
                {
                    this->tail_select(resources, root, 
                                        path_generator_type::generate(resources, last, member.key(), options), 
                                        member.value(), receiver, options);
                }
            }
            //std::cout << "end wildcard_selector\n";
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code&) const override
        {
            auto jptr = resources.create_json(json_array_arg, semantic_tag::none, resources.get_allocator());
            json_array_receiver<Json,JsonReference> receiver(jptr);
            select(resources, root, last, current, receiver, options);
            return *jptr;
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("wildcard selector");
            s.append(base_selector<Json,JsonReference>::to_string(level));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class recursive_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        recursive_selector()
            : base_selector<Json,JsonReference>()
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_array())
            {
                this->tail_select(resources, root, last, current, receiver, options);
                for (std::size_t i = 0; i < current.size(); ++i)
                {
                    select(resources, root, 
                           path_generator_type::generate(resources, last, i, options), current[i], receiver, options);
                }
            }
            else if (current.is_object())
            {
                this->tail_select(resources, root, last, current, receiver, options);
                for (auto& item : current.object_range())
                {
                    select(resources, root, 
                           path_generator_type::generate(resources, last, item.key(), options), item.value(), receiver, options);
                }
            }
            //std::cout << "end wildcard_selector\n";
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code&) const override
        {
            auto jptr = resources.create_json(json_array_arg, semantic_tag::none, resources.get_allocator());
            json_array_receiver<Json,JsonReference> receiver(jptr);
            select(resources, root, last, current, receiver, options);
            return *jptr;
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("wildcard selector");
            s.append(base_selector<Json,JsonReference>::to_string(level));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class union_selector final : public jsonpath_selector<Json,JsonReference>
    {
        using supertype = jsonpath_selector<Json,JsonReference>;
    public:
        using char_type = typename Json::char_type;
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_expression_type = path_expression<Json, JsonReference>;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;
        using selector_type = typename supertype::selector_type;
    private:
        std::vector<selector_type*> selectors_;
        selector_type* tail_;
    public:
        union_selector(std::vector<selector_type*>&& selectors)
            : supertype(true, 11), selectors_(std::move(selectors)), tail_(nullptr)
        {
        }

        void append_selector(selector_type* tail) override
        {
            if (tail_ == nullptr)
            {
                tail_ = tail;
                for (auto& selector : selectors_)
                {
                    selector->append_selector(tail);
                }
            }
            else
            {
                tail_->append_selector(tail);
            }
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current, 
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            for (auto& selector : selectors_)
            {
                selector->select(resources, root, last, current, receiver, options);
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code&) const override
        {
            auto jptr = resources.create_json(json_array_arg, semantic_tag::none, resources.get_allocator());
            json_array_receiver<Json,JsonReference> receiver(jptr);
            select(resources,root,last,current,receiver,options);
            return *jptr;
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("union selector ");
            for (auto& selector : selectors_)
            {
                s.append(selector->to_string(level+1));
                //s.push_back('\n');
            }

            return s;
        }
    };

    template <class Json,class JsonReference>
    class filter_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

        expression<Json,JsonReference> expr_;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        filter_selector(expression<Json,JsonReference>&& expr)
            : base_selector<Json,JsonReference>(), expr_(std::move(expr))
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current, 
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_array())
            {
                for (std::size_t i = 0; i < current.size(); ++i)
                {
                    std::error_code ec;
                    value_type r = expr_.evaluate(resources, root, current[i], options, ec);
                    bool t = ec ? false : detail::is_true(r);
                    if (t)
                    {
                        this->tail_select(resources, root, 
                                            path_generator_type::generate(resources, last, i, options), 
                                            current[i], receiver, options);
                    }
                }
            }
            else if (current.is_object())
            {
                for (auto& member : current.object_range())
                {
                    std::error_code ec;
                    value_type r = expr_.evaluate(resources, root, member.value(), options, ec);
                    bool t = ec ? false : detail::is_true(r);
                    if (t)
                    {
                        this->tail_select(resources, root, 
                                            path_generator_type::generate(resources, last, member.key(), options), 
                                            member.value(), receiver, options);
                    }
                }
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code&) const override
        {
            auto jptr = resources.create_json(json_array_arg, semantic_tag::none, resources.get_allocator());
            json_array_receiver<Json,JsonReference> receiver(jptr);
            select(resources, root, last, current, receiver, options);
            return *jptr;
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("filter selector ");
            s.append(expr_.to_string(level+1));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class index_expression_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;
        using allocator_type = typename Json::allocator_type;
        using string_type = typename Json::string_type;

        expression<Json,JsonReference> expr_;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        index_expression_selector(expression<Json,JsonReference>&& expr)
            : base_selector<Json,JsonReference>(), expr_(std::move(expr))
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current, 
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            std::error_code ec;
            value_type j = expr_.evaluate(resources, root, current, options, ec);

            if (!ec)
            {
                if (j.template is<std::size_t>() && current.is_array())
                {
                    std::size_t start = j.template as<std::size_t>();
                    this->tail_select(resources, root, 
                                      path_generator_type::generate(resources, last, start, options),
                                      current.at(start), receiver, options);
                }
                else if (j.is_string() && current.is_object())
                {
                    auto sv = j.as_string_view();
                    this->tail_select(resources, root, 
                                      path_generator_type::generate(resources, last, sv, options),
                                      current.at(j.as_string_view()), receiver, options);
                }
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code& ec) const override
        {
            //std::cout << "index_expression_selector current: " << current << "\n";

            value_type j = expr_.evaluate(resources, root, current, options, ec);

            if (!ec)
            {
                if (j.template is<std::size_t>() && current.is_array())
                {
                    std::size_t start = j.template as<std::size_t>();
                    return this->evaluate_tail(resources, root, last, current.at(start), options, ec);
                }
                else if (j.is_string() && current.is_object())
                {
                    return this->evaluate_tail(resources, root, last, current.at(j.as_string_view()), options, ec);
                }
                else
                {
                    return resources.null_value();
                }
            }
            else
            {
                return resources.null_value();
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("bracket expression selector ");
            s.append(expr_.to_string(level+1));
            s.append(base_selector<Json,JsonReference>::to_string(level+1));

            return s;
        }
    };

    template <class Json,class JsonReference>
    class slice_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;
        using path_generator_type = path_generator<Json, JsonReference>;

        slice slice_;
    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using node_receiver_type = typename supertype::node_receiver_type;

        slice_selector(const slice& slic)
            : base_selector<Json,JsonReference>(), slice_(slic) 
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current,
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            if (current.is_array())
            {
                auto start = slice_.get_start(current.size());
                auto end = slice_.get_stop(current.size());
                auto step = slice_.step();

                if (step > 0)
                {
                    if (start < 0)
                    {
                        start = 0;
                    }
                    if (end > static_cast<int64_t>(current.size()))
                    {
                        end = current.size();
                    }
                    for (int64_t i = start; i < end; i += step)
                    {
                        std::size_t j = static_cast<std::size_t>(i);
                        this->tail_select(resources, root, 
                                            path_generator_type::generate(resources, last, j, options), 
                                            current[j], receiver, options);
                    }
                }
                else if (step < 0)
                {
                    if (start >= static_cast<int64_t>(current.size()))
                    {
                        start = static_cast<int64_t>(current.size()) - 1;
                    }
                    if (end < -1)
                    {
                        end = -1;
                    }
                    for (int64_t i = start; i > end; i += step)
                    {
                        std::size_t j = static_cast<std::size_t>(i);
                        if (j < current.size())
                        {
                            this->tail_select(resources, root, 
                                                path_generator_type::generate(resources, last,j,options), current[j], receiver, options);
                        }
                    }
                }
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code&) const override
        {
            auto jptr = resources.create_json(json_array_arg, semantic_tag::none, resources.get_allocator());
            json_array_receiver<Json,JsonReference> accum(jptr);
            select(resources, root, last, current, accum, options);
            return *jptr;
        }
    };

    template <class Json,class JsonReference>
    class function_selector final : public base_selector<Json,JsonReference>
    {
        using supertype = base_selector<Json,JsonReference>;

        expression<Json,JsonReference> expr_;

    public:
        using value_type = typename supertype::value_type;
        using reference = typename supertype::reference;
        using pointer = typename supertype::pointer;
        using path_value_pair_type = typename supertype::path_value_pair_type;
        using path_node_type = typename supertype::path_node_type;
        using path_generator_type = path_generator<Json,JsonReference>;
        using node_receiver_type = typename supertype::node_receiver_type;

        function_selector(expression<Json,JsonReference>&& expr)
            : base_selector<Json,JsonReference>(), expr_(std::move(expr))
        {
        }

        void select(dynamic_resources<Json,JsonReference>& resources,
                    reference root,
                    const path_node_type& last, 
                    reference current, 
                    node_receiver_type& receiver,
                    result_options options) const override
        {
            std::error_code ec;
            value_type ref = expr_.evaluate(resources, root, current, options, ec);
            if (!ec)
            {
                this->tail_select(resources, root, last, *resources.create_json(std::move(ref)), receiver, options);
            }
        }

        reference evaluate(dynamic_resources<Json,JsonReference>& resources,
                           reference root,
                           const path_node_type& last, 
                           reference current, 
                           result_options options,
                           std::error_code& ec) const override
        {
            value_type ref = expr_.evaluate(resources, root, current, options, ec);
            if (!ec)
            {
                return this->evaluate_tail(resources, root, last, *resources.create_json(std::move(ref)), 
                                    options, ec);
            }
            else
            {
                return resources.null_value();
            }
        }

        std::string to_string(int level = 0) const override
        {
            std::string s;
            if (level > 0)
            {
                s.append("\n");
                s.append(level*2, ' ');
            }
            s.append("function_selector ");
            s.append(expr_.to_string(level+1));

            return s;
        }
    };

} // namespace detail
} // namespace jsonpath
} // namespace jsoncons

#endif
